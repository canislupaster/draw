#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>

#include <stdio.h>
#include <thread>
#include <stop_token>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <optional>

class SerialPortException: public std::runtime_error {
public:
	SerialPortException(const std::string &message)
		: std::runtime_error(message) {
	}
};

class ArduinoSerial {
	std::mutex recv_running;
	std::mutex buf_mut;
	std::condition_variable buf_cond;
	bool stop=false;
	std::string buf;
	bool rd_err=false;

	int fd;
	termios toptions;

	void check() {
		int flags = fcntl(fd, F_GETFL, 0);
		if (flags<0 || errno == EBADF) {
			throw SerialPortException("Connection error");
		}

		if ((flags & O_RDWR) == 0) {
			throw SerialPortException("File descriptor is not open for reading and writing.");
		}
	}
public:
	ArduinoSerial(const char *serialport, int baud) {
		fd = open(serialport, O_RDWR | O_NOCTTY);
		if (fd == -1) {
			throw SerialPortException("Failed to open serial port.");
		}

		if (tcgetattr(fd, &toptions) < 0) {
			close(fd);
			throw SerialPortException("Failed to get term attributes.");
		}

		speed_t brate = baud;
		switch (baud) {
		case 4800: brate = B4800;
			break;
		case 9600: brate = B9600;
			break;
#ifdef B14400
		case 14400: brate = B14400;
			break;
#endif
		case 19200: brate = B19200;
			break;
#ifdef B28800
		case 28800: brate = B28800;
			break;
#endif
		case 38400: brate = B38400;
			break;
		case 57600: brate = B57600;
			break;
		case 115200: brate = B115200;
			break;
		}

		cfsetispeed(&toptions, brate);
		cfsetospeed(&toptions, brate);

		toptions.c_cflag &= ~(PARENB | PARODD);
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;

		toptions.c_cflag |= CREAD | CLOCAL;
		toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
		toptions.c_iflag &= ~(INPCK | ISTRIP);

		toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
		toptions.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);

		toptions.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		toptions.c_oflag &= ~OPOST;
		toptions.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		toptions.c_cflag &= ~(CSIZE | PARENB);
		toptions.c_cflag |= CS8;

		toptions.c_cc[VMIN] = 0;
		toptions.c_cc[VTIME] = 10;

		if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
			close(fd);
			throw SerialPortException("Failed to set term attributes.");
		}

		//the best solution i could find :/
		std::thread([this]() {
			std::lock_guard recv_lock(recv_running);
			std::string tmp(1024, 0);
			while (true) {
				int r = read(fd, tmp.data(), tmp.size());

				std::lock_guard buf_lock(buf_mut);
				if (r==-1) rd_err=true;
				else if (r>0) {
					buf.append(tmp.substr(0,r));
				}

				if (r!=0) buf_cond.notify_all();
				if (stop) break;
			}
		}).detach();
	}

	~ArduinoSerial() {
		{
			std::lock_guard buf_lock(buf_mut);
			stop=true;
		}

		recv_running.lock();
		if (fd != -1) close(fd);
	}

	void writeByte(uint8_t b) {
		int n = write(fd, &b, 1);
		if (n != 1)
			throw SerialPortException("Failed to write byte to serial port.");
		check();
	}

	void writeStr(const char *str) {
		int len = strlen(str);
		int n = write(fd, str, len);
		if (n != len)
			throw SerialPortException("Failed to write data to serial port.");

		tcdrain(fd);
		usleep(1000*100); //lmfao, small delay between writes is apparently necessary

		check();
	}

	void flushInput() {
		std::unique_lock buf_lock(buf_mut);
		buf.clear();
	}

	std::optional<std::string> readUntil(char until, int ms) {
		check();

		int bufi=0;
		auto dur = std::chrono::milliseconds(ms);
		auto then = std::chrono::high_resolution_clock::now()+dur;

		std::unique_lock buf_lock(buf_mut);

		for (bool wait=false; ; wait=true) {
			auto now = std::chrono::high_resolution_clock::now();
			if (now >= then) return std::nullopt;

			if (wait) buf_cond.wait_for(buf_lock, then-now);

			while (bufi<buf.size() && buf[bufi]!=until) bufi++;

			if (bufi<buf.size() && buf[bufi]==until) {
				auto ret = buf.substr(0, bufi+1);
				buf.erase(0, bufi+1);
				return ret;
			}
		}
	}
};

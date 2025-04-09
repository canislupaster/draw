#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <cstring>

#include <stdio.h>
#include <thread>
#include <vector>
#include <exception>
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
	std::mutex buf_mut;
	std::condition_variable buf_cond;
	bool stop=false;
	std::string buf;

	std::thread recv, send;
	std::condition_variable send_cond;
	std::mutex send_mut;
	std::vector<std::string> send_buf;

	std::optional<std::exception_ptr> err;

	int fd;
	termios toptions;

	void seterr(std::exception_ptr e) {
		std::scoped_lock lock(buf_mut, send_mut);
		err = e;
		stop=true;
	}

	void check() {
		int flags = fcntl(fd, F_GETFL, 0);
		if (flags<0 || errno == EBADF) {
			throw SerialPortException("Connection error");
		}

		if ((flags & O_RDWR) == 0) {
			throw SerialPortException("File descriptor is not open for reading and writing.");
		}

		if (err) std::rethrow_exception(*err);
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

		//supposed to use standard speeds defined in termios.h i think but whatever
		cfsetispeed(&toptions, baud);
		cfsetospeed(&toptions, baud);

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
		recv = std::thread([this]() {
			try {
				std::string tmp(1024, 0);

				while (true) {
					int r = read(fd, tmp.data(), tmp.size());

					std::unique_lock buf_lock(buf_mut);
					if (r==-1) {
						throw SerialPortException("Failed to read data from serial port.");
					} else if (r>0) {
						buf.append(tmp.substr(0,r));
					}

					if (r!=0) buf_cond.notify_all();
					if (stop) break;
				}
			} catch (...) {
				seterr(std::current_exception());
			}
		});

		send = std::thread([this]() {
			try {
				std::unique_lock lock(send_mut);

				std::vector<std::string> tmp;
				while (!stop) {
					tmp.swap(send_buf);
					lock.unlock();

					for (auto& str : tmp) {
						int n = write(fd, str.c_str(), str.size());
						if (n != str.size()) {
							lock.unlock();
							throw SerialPortException("Failed to write data to serial port.");
							break;
						}

						tcdrain(fd);
						//lmfao, small delay between writes is apparently necessary
						//its ok we have a thread for that
						usleep(1000*100);
					}

					tmp.clear();

					lock.lock();
					if (send_buf.size()) continue;

					send_cond.wait(lock);
				}
			} catch (...) {
				seterr(std::current_exception());
			}
		});
	}

	~ArduinoSerial() {
		{
			std::scoped_lock lock(buf_mut, send_mut);
			stop=true;
		}

		send_cond.notify_all();
		recv.join(), send.join();
		if (fd != -1) close(fd);
	}

	void writeStr(std::string const& str) {
		std::unique_lock lock(send_mut);
		check();

		send_buf.push_back(str);
		send_cond.notify_all();
	}

	void discard() {
		std::scoped_lock lock(buf_mut, send_mut);
		buf.clear(), send_buf.clear();
	}

	std::optional<std::string> readUntil(char until, int ms) {
		std::unique_lock buf_lock(buf_mut);

		check();

		int bufi=0;
		auto dur = std::chrono::milliseconds(ms);
		auto then = std::chrono::high_resolution_clock::now()+dur;

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

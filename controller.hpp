#include "point.hpp"
#include "serial.hpp"

#include <deque>
#include <exception>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

#include "path.hpp"

using namespace std;

struct Controller {
	struct Stat {
		Pt current, from, to;
		bool drawing;
	};

	const int max_log = 20;
	const int move_lookahead = 8;

	struct State {
		enum {Idle, Running, Paused, PausedForColorChange} ty = Idle;
		optional<uint32_t> next_color;

		bool locked=false, jogging=false; //jog whenever not running

		optional<Stat> last_stat;
		optional<exception_ptr> err;
		deque<string> log;

		bool was_running() const { return ty!=Idle; }
		State() = default;
	};

	Controller(int baud) {
		filesystem::directory_iterator dirit("/dev/");
		auto dirit_end = filesystem::end(dirit);
		auto ent = find_if(dirit, dirit_end, [](filesystem::directory_entry const& can) {
			return can.path().filename().string().starts_with("cu.usb");
		});

		if (ent==dirit_end)
			throw runtime_error("device not found");

		serial = make_unique<ArduinoSerial>(ent->path().c_str(), baud);

    io_thread = thread([this] {
	    try {
	      recv();
	    } catch (...) {
        unique_lock guard(mut);
				state.err = current_exception();
	    }
    });
	}

	struct Guard {
		void jog(Pt where) {
			if (c.state.ty!=State::Running) {
				c.state.jogging=true;
				ostringstream oss;
				oss<<"go "<<lround(where.x)<<","<<lround(where.y)<<"\n";
				c.serial->writeStr(oss.str());
			}
		}

		optional<std::exception_ptr> err() {
			if (c.state.err) {
				auto v = *c.state.err;
				c.state.err.reset();
				return v;
			}

			return nullopt;
		}

		void set_pen_amt(float x) {
			if (c.state.ty!=State::Running) {
				ostringstream oss;
				oss<<"pen "<<lround(x*100.0f)<<"\n";
				c.serial->writeStr(oss.str());
			}
		}

		void halt() {
			if (c.state.jogging) {
				c.state.jogging=false;
				c.serial->writeStr("halt\n");
			}
		}

		void cancel(bool reset) {
			if (c.state.ty!=State::Idle || reset)
				c.serial->writeStr(reset ? "reset\n" : "cancel\n");
			c.state.ty=State::Idle;
		}

		void pause() {
			if (c.state.ty==State::Running) {
				c.state.ty=State::Paused;
				c.serial->writeStr("pause\n");
			}
		}

		void resume() {
			if (c.state.ty==State::Paused) {
				c.state.ty=State::Running;
				c.serial->writeStr("resume\n");
			} else if (c.state.ty==State::PausedForColorChange) {
				c.state.ty=State::Running;
			}

			//either resumes or sends next move, in either case jogging is stopped
			c.state.jogging=false;
		}

		void start_paths(span<const Path> paths_) {
			if (c.state.ty==State::Idle) {
				c.state.ty=State::Running;
				c.paths.assign(paths_.begin(), paths_.end());
				c.reload_path=true;
			}
		}

		void do_lock() {
			c.state.locked=true;
			c.serial->writeStr("lock\n");
		}

		void unlock() {
			c.state.locked=false;
			c.serial->writeStr("unlock\n");
		}

		void send(string const& cmd) {
			c.serial->writeStr(cmd);
		}

		bool paused() const {
			return c.state.ty==State::Paused || c.state.ty==State::PausedForColorChange;
		}

		State const& operator()() const { return c.state; }
		State const* operator->() const { return &c.state; }
		State const& get() const { return c.state; }

	private:
		unique_lock<mutex> lock;
		Controller& c;

		Guard(unique_lock<mutex> lock, Controller& c): lock(std::move(lock)), c(c) {}

		friend Controller;
	};

	Guard guard() {
		return {unique_lock<mutex>(mut), *this};
	}

	void recv() {
		cout<<"started I/O thread"<<endl;
		unique_lock lock(mut);

		serial->discard();

		int move_n=0, move_done=0;
		int pi=0, ci=0;
		bool is_init=false; //wait for first status update before doing shit
		optional<uint32_t> last_color;

		string ln;
		while (!quit) {
			if (reload_path) {
				move_n=move_done=pi=ci=0;
				reload_path=false;
				if (paths.back().color!=paths[0].color) last_color.reset();
				else last_color=paths[0].color;
			}

			while (is_init && state.ty==State::Running && move_n-move_done<move_lookahead) {
				if (pi==paths.size()) {
					if (move_n==move_done) state.ty=State::Idle;
					break;
				}

				if (!last_color || paths[pi].color!=*last_color) {
					if (move_n==move_done) {
						state.ty=State::PausedForColorChange;
						state.next_color = paths[pi].color;
						last_color = paths[pi].color;
					}

					break;
				}

				ostringstream oss;

				auto const& cmd = paths[pi].cmds[ci++];
				if (cmd.ty==DrawCmd::Line) oss<<"l ";
				for (int j=0; j<(cmd.ty==DrawCmd::Line ? 2 : 4); j++)
					oss<<lround(cmd[j].x)<<","<<lround(cmd[j].y)<<" ";

				if (ci==paths[pi].cmds.size()) pi++, ci=0;
				else oss<<"stay_down";
				oss<<"\n";

				cout<<oss.str();
				serial->writeStr(oss.str());

				move_n++;
			}

			lock.unlock();
			auto optln = serial->readUntil('\n', 200);
			lock.lock();

			if (!optln) continue;

			ln = *optln;
			ln.erase(0, ln.find_first_not_of(" \n\r\t"));
			ln.erase(ln.find_last_not_of(" \n\r\t")+1);
			
			cerr<<ln<<endl;

			is_init=true;

			if (ln=="done") {
				move_done++;
			} else if (ln.starts_with("STATE")) {
				Stat newstat;
				int newstat_drawing;
				if (sscanf(ln.c_str(), "STATE (%f, %f) (%f, %f) (%f, %f) %i",
					&newstat.current.x, &newstat.current.y,
					&newstat.from.x, &newstat.from.y, &newstat.to.x, &newstat.to.y, &newstat_drawing)==7) {

					newstat.drawing = newstat_drawing==1;
					state.last_stat = newstat;
				}
			}

			state.log.emplace_back(std::move(ln));
			while (state.log.size()>max_log) state.log.pop_front();
		}
	}

	~Controller() {
		quit=true;
		io_thread.join();
	}

private:
	unique_ptr<ArduinoSerial> serial;
	State state;
	mutex mut;
	bool quit=false, reload_path=false;
	vector<Path> paths;
	thread io_thread;
};
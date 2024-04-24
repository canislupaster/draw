#include "point.hpp"
#include "serial.hpp"

#include <deque>
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
	const int move_lookahead = 3;

	struct State {
		bool running, was_running, cancelled, locked;
		optional<Stat> last_stat;
		shared_ptr<exception> err;
		deque<string> log;

		State(): running(false), was_running(false), cancelled(false), locked(false) {}
	};

	Controller() {
		filesystem::directory_iterator dirit("/dev/");
		auto dirit_end = filesystem::end(dirit);
		auto ent = find_if(dirit, dirit_end, [](filesystem::directory_entry const& can) {
			return can.path().filename().string().starts_with("cu.usbserial");
		});

		if (ent==dirit_end)
			throw runtime_error("device not found");

		serial = make_unique<ArduinoSerial>(ent->path().c_str(), 9600);
	}

	struct Guard {
		void pause() {
			if (state.running) {
				state.running=false;
				serial.writeStr("pause\n");
			}
		}

		void cancel(bool reset) {
			state.running=false, state.cancelled=true;
			serial.writeStr(reset ? "reset\n" : "cancel\n");
		}

		void resume() {
			if (!state.running && state.was_running) {
				state.running=true;
				serial.writeStr("resume\n");
			}
		}

		void do_lock() {
			state.locked=true;
			serial.writeStr("lock\n");
		}

		void unlock() {
			state.locked=false;
			serial.writeStr("unlock\n");
		}

		bool paused() const { return !state.running && !state.cancelled && state.was_running; }

		State const& operator()() const { return state; }
		State const* operator->() const { return &state; }
		State const& get() const { return state; }

	private:
		unique_lock<mutex> lock;
		State& state;
		ArduinoSerial& serial;

		Guard(unique_lock<mutex> lock, State& state, ArduinoSerial& serial): lock(std::move(lock)), state(state), serial(serial) {}

		friend Controller;
	};

	Guard guard() {
		return Guard(unique_lock<mutex>(mut), state, *serial);
	}

	void recv(vector<Path> const& paths) {
		cout<<"started"<<endl;
		serial->flushInput();
		serial->writeStr("\n"); //i dont know why this needs to be here but it's clearly termios's problem

		int move_n=0, move_done=0;

		for (int pi=0; pi<paths.size(); pi++) {
			for (DrawCmd const& cmd: paths[pi].cmds) {
				ostringstream oss;

				if (cmd.ty==DrawCmd::Line) oss<<"l ";
				for (Pt p: cmd.pts)
					for (int j=0; j<4; j++) oss<<lround(p.x)<<","<<lround(p.y)<<" ";

				bool end = &cmd==&paths[pi].cmds.back();
				if (!end) oss<<"stay_down";
				oss<<"\n";

				cout<<oss.str()<<endl;

				{
					lock_guard guard(mut);
					serial->writeStr(oss.str().c_str());
				}

				move_n++;

				int look = end && pi==paths.size()-1 ? 1 : move_lookahead;

				string ln;
				while (move_n - move_done >= look) {
					auto optln = serial->readUntil('\n', 200);

					lock_guard guard(mut);
					if (state.cancelled) return;
					if (!optln) continue;

					ln = *optln;
					ln.erase(0, ln.find_first_not_of(" \n\r\t"));
					ln.erase(ln.find_last_not_of(" \n\r\t")+1);
					
					cerr<<ln<<endl;

					if (ln=="done") {
						move_done++;
						break;
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
		}
	}

	static void start(shared_ptr<Controller> ptr, vector<Path> const& paths) {
		lock_guard guard(ptr->mut);
		ptr->state.running=ptr->state.was_running=true, ptr->state.cancelled=false;

    thread([ptr,paths] {
			unique_lock<mutex> lock(ptr->running_lock);

	    try {
	      ptr->recv(paths);
	    } catch (std::exception const& ex) {
        lock_guard guard(ptr->mut);
        ptr->state.err = make_shared<std::exception>(ex);
	    }

    	lock_guard guard(ptr->mut);
			ptr->state.running=ptr->state.was_running=false;
    }).detach();
	}

	~Controller() {
		{
			lock_guard guard(mut);
			state.cancelled=true;
		}

		running_lock.lock();
		serial->writeStr("cancel\n");
	}

private:
	unique_ptr<ArduinoSerial> serial;
	State state;
	mutex mut, running_lock;
};
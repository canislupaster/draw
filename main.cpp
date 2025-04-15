#include <chrono>
#include <cstddef>
#include <exception>
#include <iostream>
#include <algorithm>
#include <vector>
#include <fstream>

#include "imgui.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

#include "cereal/archives/json.hpp"
#include "cereal/types/optional.hpp"
#include "cereal/types/string.hpp"
#include "imgui_internal.h"

#define NFD_THROWS_EXCEPTIONS
#include "nfd.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "point.hpp"
#include "controller.hpp"
#include "path.hpp"
#include "style.hpp"

using namespace std;

// void size_cb(ImGuiSizeCallbackData* data) {
// 	auto& cur_sz = data->DesiredSize;
// 	float plot_bound_sc = min(cur_sz.x/dim.x, cur_sz.y/dim.y);
// 	if (abs(dim.y*cur_sz.x - dim.x*cur_sz.y)/max(dim.x, dim.y) > 1) {
// 		cur_sz = (dim*plot_bound_sc).imvec();
// 	}
// }

struct Texture {
	GLuint gl_tex;
	ImTextureID texture;
	int width, height;

	~Texture() {
		glDeleteTextures(1, &gl_tex);
	}
};

constexpr string RESOURCE_ROOT = "./resources";

Texture load_image(string const& filename) {
	int image_width = 0;
	int image_height = 0;

	unsigned char* image_data = stbi_load(
		(filesystem::path(RESOURCE_ROOT) / filename).c_str(),
		&image_width, &image_height, nullptr, 4
	);

	if (image_data == nullptr)
		throw runtime_error("failed to load image");

	// Create a OpenGL texture identifier
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

	// Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
	stbi_image_free(image_data);

	return Texture { image_texture, (void*)(intptr_t)image_texture, image_width, image_height };
}

template<class T, size_t period=400>
struct Debounce {
	T v, lv;
	optional<chrono::steady_clock::time_point> last_change;
	chrono::milliseconds p;

	Debounce(T iv): v(iv), lv(iv), p(period) {}

	bool operator()() {
		if (v!=lv) {
			lv=v;
			last_change = chrono::steady_clock::now();
			return false;
		} else if (last_change && chrono::steady_clock::now()-*last_change > p) {
			last_change.reset();
			return true;
		} else {
			return false;
		}
	}
};

struct Save {
	int baud=230400;
	optional<Pt> cursor;
	float slider_time=1.0, slider_overlay_scale=0.5,
		slider_path_scale=0.5,
		zoom=1.0, zoom_before=1.0;
	bool original_colors=false;
	Pt pan_center = dim/2;
	optional<string> path;
	float fill_angle=0.0;
	float fill_dist=45.0;

	template<class Archive>
	void serialize(Archive& archive) {
		archive(CEREAL_NVP(baud), CEREAL_NVP(cursor), CEREAL_NVP(slider_time), CEREAL_NVP(slider_overlay_scale),
			CEREAL_NVP(slider_path_scale), CEREAL_NVP(zoom), CEREAL_NVP(zoom_before),
			CEREAL_NVP(original_colors), CEREAL_NVP(pan_center), CEREAL_NVP(path),
			CEREAL_NVP(fill_angle), CEREAL_NVP(fill_dist));
	}
};

int main() {
	auto sdlerr = []() {
		auto e = SDL_GetError();
		if (e && strlen(e)>0) throw runtime_error(e);
	};

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) sdlerr();

	const char* glsl_version = "#version 150";
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
	auto window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window* window = SDL_CreateWindow("Plotter", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
	if (window == nullptr) sdlerr();

	SDL_GLContext gl_context = SDL_GL_CreateContext(window);
	SDL_GL_MakeCurrent(window, gl_context);
	SDL_GL_SetSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

	int display = SDL_GetWindowDisplayIndex(window);

	float dpi;
	SDL_GetDisplayDPI(display, &dpi, nullptr, nullptr);

	ImFontConfig config;
	config.RasterizerDensity = dpi/120;
	auto roboto = io.Fonts->AddFontFromFileTTF("Roboto-Regular.ttf", ceil(16*dpi/226), &config);

	ImGui::StyleColorsDark();
	embraceTheDarkness();

	ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
	ImGui_ImplOpenGL3_Init(glsl_version);

	SDL_ShowWindow(window);

	NFD::Guard nfdGuard;

	constexpr array<ImU32, 5> palette {
		IM_COL32(255, 255, 255, 255), // White
		IM_COL32(0, 255, 0, 255),     // Green
		IM_COL32(0, 0, 255, 255),     // Blue
		IM_COL32(255, 255, 0, 255),   // Yellow
		IM_COL32(255, 0, 255, 255)    // Magenta
	};

	shared_ptr<Controller> cur_control;
	optional<string> err;
	optional<Plot> plot;
	std::optional<ImVec2> wind_pos;
	Debounce<float> pen_amt(0);

	bool lock=false;

	string serial_input(256, '\0');

	const string settings_path = "./settings.json";
	ifstream fin(settings_path);
	Save save;
	float other_time_slider=save.slider_time;

	auto do_save = [&save, settings_path]() {
		cout<<"saving..."<<endl;

		ofstream fout(settings_path);
		auto archive = cereal::JSONOutputArchive(fout);
		archive(save);
	};

	if (fin.is_open()) {
		auto archive = cereal::JSONInputArchive(fin);
		archive(save);
		if (save.path) try {
			plot.emplace(*save.path, save.fill_angle, save.fill_dist);
		} catch (exception const& ex) {
			err = ex.what();
		}
	}

	auto draw_image = load_image("draw.png"),
		move_image = load_image("move.png"),
		idle_image = load_image("idle.png"),
		cursor_image = load_image("cursor.png");

	bool err_open=false;

	uint64_t fps_ticks = SDL_GetPerformanceFrequency()/60, last_swap = 0;

	bool done=false;
	while (!done) {
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			ImGui_ImplSDL2_ProcessEvent(&event);
			if (event.type == SDL_QUIT || (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
				done = true;
		}

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();
		ImGui::PushFont(roboto);

		auto viewport = ImGui::GetMainViewport();
		ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(viewport, ImGuiDockNodeFlags_PassthruCentralNode);

		ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_None);

		if (ImGui::CollapsingHeader("Preview options",ImGuiTreeNodeFlags_DefaultOpen)) {
			if (!plot) {
				ImGui::Text("Load an SVG to see preview");
			}

			//lmfao
			if (ImGui::SliderFloat("Time", &other_time_slider, 0.0,1.0))
				save.slider_time = other_time_slider;
			if (ImGui::DragFloat("Time (fine)", &save.slider_time, 25.0f/plot->total_time, 0.0,1.0))
				other_time_slider = save.slider_time;

			ImGui::SliderFloat("Scale overlay", &save.slider_overlay_scale, 0.0, 1.0);
			ImGui::SliderFloat("Scale path", &save.slider_path_scale, 0.0, 1.0);

			ImGui::DragFloat("Zoom", &save.zoom, 0.1, 0.1, 50.0);

			if (ImGui::Button("Reset position")) {
				save.pan_center = dim/2;
			}

			ImGui::Checkbox("Show original colors", &save.original_colors);
		}

		if (err && !err_open) {
			err_open=true;
			ImGui::OpenPopup("Error");
		}

		if (ImGui::BeginPopupModal("Error", &err_open, ImGuiWindowFlags_AlwaysAutoResize)) {
			ImGui::TextColored(ImVec4(1.0,0,0,1.0), "Controller error:");
			ImGui::TextWrapped("%s", err->c_str());
			ImGui::EndPopup();
		}

		if (!err_open) err.reset();

		optional<Controller::Guard> state;
		try {
			if (ImGui::CollapsingHeader("Controls",ImGuiTreeNodeFlags_DefaultOpen)) {
				if (cur_control) {
					auto& g = state.emplace(std::move(cur_control->guard()));
					if (auto e = g.err(); e) {
						try {
							rethrow_exception(*e);
						} catch (std::exception const& ex) {
							err = ex.what();
						}
					}

					if (g->ty!=Controller::State::Idle) {
						if (g->ty==Controller::State::PausedForColorChange) {
							ImGui::Text("Change pen color to:");

							auto list = ImGui::GetWindowDrawList();
							auto pos = ImGui::GetCursorScreenPos();

							const float sz = 40.0, pad=5.0;
							pos.x+=pad; pos.y+=pad;
							list->AddRectFilled(pos, ImVec2(pos.x+sz,pos.y+sz), *g->next_color);
							ImGui::SetCursorScreenPos(ImVec2(pos.x-pad, pos.y+sz+pad));
						}

						ImGui::Text(g.paused() ? "PAUSED" : "DRAWING");
						if (ImGui::Button(g.paused() ? "Resume" : "Stop")) {
							if (g.paused()) g.resume(); else g.pause();
						}
					} else if (plot && ImGui::Button("Start")) {
						g.start_paths(plot->paths);
					}

					ImGui::SameLine();
					if (ImGui::Button("Cancel")) g.cancel(false);
					ImGui::SameLine();
					if (ImGui::Button("Reset")) g.cancel(true);

					ImGui::Checkbox("Lock", &lock);
					if (lock!=g->locked) lock ? g.do_lock() : g.unlock();

					if (g->ty==Controller::State::Idle && ImGui::Button("Disconnect")) {
						state.reset();
						cur_control.reset();
					}
				} else {
					ImGui::InputInt("Baud rate", &save.baud);

					if (ImGui::Button("Connect")) {
						cur_control = make_shared<Controller>(save.baud);
						lock = cur_control->guard()->locked;
					}
				}
			}

			if ((!state || state->get().ty==Controller::State::Idle)
				&& ImGui::CollapsingHeader("Import",ImGuiTreeNodeFlags_DefaultOpen)) {

				ImGui::SliderAngle("Fill angle", &save.fill_angle,0,180);
				ImGui::InputFloat("Fill distance", &save.fill_dist, 5.0);

				if (save.path) {
					ImGui::TextWrapped("Loaded %s", save.path->c_str());
				}

				bool reload = save.path && ImGui::Button("Reload");
				if (reload || ImGui::Button("Load SVG")) {
					NFD::UniquePath outPath;
					nfdfilteritem_t filterItem = {"SVG", "svg"};

					if (reload || NFD::OpenDialog(outPath, &filterItem, 1)==NFD_OKAY) {
						std::string path = reload ? *save.path : outPath.get();

						plot.emplace(path, save.fill_angle, save.fill_dist);

						save.path = path;
						do_save();
					}
				}
			}

			if ((state && state->get().ty!=Controller::State::Running)
				&& ImGui::CollapsingHeader("Jog",ImGuiTreeNodeFlags_DefaultOpen)) {
				if (save.cursor) {
					ImGui::Text("X %.2f, Y %.2f", save.cursor->x, save.cursor->y);
					if (ImGui::Button("Jog to cursor"))
						state->jog(*save.cursor);
				} else ImGui::Text("Click on preview to set cursor");

				ImGui::SliderFloat("Pen", &pen_amt.v, 0.0, 1.0);
				if (pen_amt()) state->set_pen_amt(pen_amt.v);

				if (state->get().jogging && ImGui::Button("Stop jogging"))
					state->halt();
			}
		} catch (exception const& ex) {
			err = ex.what();
		}

		ImGui::End();

		ImGui::Begin("Logs", nullptr, ImGuiWindowFlags_None);

    if (state) {
			for (string const& log : state->get().log) {
				ImGui::TextWrapped("%s", log.c_str());
			}

			ImGui::InputText("Message", serial_input.data(), serial_input.size());
			ImGui::SameLine();
			if (ImGui::SmallButton("Send")) {
				string msg(serial_input.c_str());
				msg.push_back('\n');
				state->send(msg);
			}
		} else {
			ImGui::Text("No logs yet.");
		}

    ImGui::End();

		// ImGui::SetNextWindowSizeConstraints(ImVec2(1,1), ImGui::GetMainViewport()->Size, size_cb);
		// if (wind_pos) ImGui::SetNextWindowPos(*wind_pos);
		// ImGui::Begin("Preview", nullptr, ImGuiWindowFlags_None);

		if (plot) {
			auto node = ImGui::DockBuilderGetCentralNode(dockspace_id);

			auto cur_sz = node->Size;
			ImVec2 windowPosIm = node->Pos;
			Pt windowPos(windowPosIm);

			float plot_bound_sc = min(cur_sz.x/dim.x, cur_sz.y/dim.y);

			auto drawlist = ImGui::GetBackgroundDrawList(viewport);

			auto affine_trans = windowPos + (dim/2 - save.pan_center*save.zoom)*plot_bound_sc;
			float trans_mul = plot_bound_sc*save.zoom*dpi/226.0f;

			float overlay_thick = save.slider_overlay_scale*dpi/226.0f;

			float path_thick1 = 4.0f*save.slider_path_scale*dpi/226.0f;
			float path_thick2 = 5.0f*save.slider_path_scale*dpi/226.0f;

			auto trans = [windowPos, trans_mul, affine_trans](Pt x) {
				return (x*trans_mul + affine_trans).imvec();
			};

			auto render_cmd = [&drawlist, &save, trans, &palette, &plot, path_thick1, path_thick2](DrawCmd const& c, int pi) {
				uint32_t col = save.original_colors ? plot->paths[pi].color : palette[pi%palette.size()];

				if (c.ty==DrawCmd::Cubic)
					drawlist->AddBezierCubic(trans(c.pts[0]), trans(c.pts[1]),
						trans(c.pts[2]), trans(c.pts[3]),
						col, path_thick2);
				else
					drawlist->AddLine(trans(c.pts[0]), trans(c.pts[1]),
						col, path_thick2);
			};

			drawlist->AddRect(trans(Pt(0,0)), trans(dim), IM_COL32(255,255,0,255), 0, 0,
				3.0f*overlay_thick);

			// ImGui::InvisibleButton("previewBg", ImGui::GetContentRegionAvail());
			if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
				// wind_pos = windowPosIm;

				Pt del = ImGui::GetMouseDragDelta();
				if (ImGui::IsKeyDown(ImGuiKey_LeftShift)) {
					Pt pos = ImGui::GetMousePos(), center = windowPos + Pt(cur_sz)/2;
					Pt d_center = (pos-del-center).unit();
					d_center = d_center*(pos-center).unit().dot(d_center)*(10.0f/Pt(cur_sz).norm());

					float fac = del.dot(d_center);
					save.zoom = save.zoom_before*clamp((1+fac/2), 0.01f, 50.0f);

					drawlist->AddLine(pos.imvec(), center.imvec(), IM_COL32(255,0,0,255), 2.0f*overlay_thick);
				} else {
					save.pan_center = save.pan_center - del/trans_mul;
					ImGui::ResetMouseDragDelta();
				}
			} else {
				wind_pos.reset();
				save.zoom_before = save.zoom;
			}

			if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
				auto pos = ImGui::GetMousePos();
				save.cursor = (Pt(pos) - affine_trans)/trans_mul;
			}

			Pt last;
			auto ft = plot->find_t(save.slider_time*plot->total_time);

			for (int pi=0; pi<min((int)plot->paths.size(), ft.path_i+1); pi++) {
				auto const& c = plot->paths[pi].cmds;
				int max_cmdi = ft.path_i==pi ? ft.cmd_i : c.size();
				for (int i=0; i<max_cmdi; i++) {
					render_cmd(c[i], pi);
				}

				if (max_cmdi!=-1 && max_cmdi<c.size()) {
					auto np = c[max_cmdi].trim(0, ft.cmd_t);
					render_cmd(np, pi);
				}

				for (int i=0; i<(ft.path_i==pi ? ft.cmd_i+1 : c.size()); i++) {
					drawlist->AddCircleFilled(trans(c[i].start()),
						8.0f*overlay_thick, IM_COL32(255,255,0,255));
				}

				if (max_cmdi==c.size()) {
					drawlist->AddCircleFilled(trans(c.back().end()),
						8.0f*overlay_thick, IM_COL32(255,255,0,255));
				}

				if (pi>0) {
					const float dash_len = maxw/70.0/save.zoom;
					Pt dif = plot->paths[pi].start() - last;
					float tot = dif.norm(), inc=dash_len/tot;

					Pt prev = last; bool d=true;
					for (float t=0.0; t<1.0; t+=inc, d=!d) {
						Pt nxt = last + dif*min(t+inc, 1.0f);
						if (d || t+inc>=1.0)
							drawlist->AddLine(trans(prev), trans(nxt), IM_COL32_WHITE, path_thick1);
						prev=nxt;
					}
				}

				last = plot->paths[pi].end();
			}

			drawlist->AddCircleFilled(trans(ft.pt), 12.0*overlay_thick, IM_COL32(0, 255, 255, 255));

			if (state && state->get().last_stat) {
				auto [current, from, to, is_drawing] = *state->get().last_stat;
				float tri_w = 26.0*overlay_thick/trans_mul;

				Pt dif = to - from;
				bool still = dif==Pt(0,0);
				if (still) dif={tri_w,0};
				else dif=dif.unit()*tri_w;

				Pt perp = Pt(-dif.y, dif.x);

				Pt up_cur = current+dif;
				current=current-dif;
				Pt l = current - perp, r = current+perp,
					ul = up_cur - perp, ur = up_cur+perp;

				drawlist->AddImageQuad(is_drawing ? draw_image.texture : (still ? idle_image.texture : move_image.texture),
					trans(l), trans(r), trans(ur), trans(ul),
					ImVec2(1,0), ImVec2(1,1), ImVec2(0,1), ImVec2(0,0),
					IM_COL32(255, 0, 0, 255));

				if (!still) drawlist->AddLine(trans(from), trans(to),
					IM_COL32(255, 0, 0, 255), 6.0f*overlay_thick);
			}

			float cursor_w = 30.0f*overlay_thick/trans_mul;
			if (save.cursor) {
				Pt off(cursor_w/2, cursor_w/2);
				drawlist->AddImage(cursor_image.texture,
					trans(*save.cursor-off), trans(*save.cursor+off),
					ImVec2(0,0), ImVec2(1,1));
			}
		}

		ImGui::PopFont();
		ImGui::Render();

		glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
		glClearColor(0.0,0.0,0.0,1.0);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(window);

		uint64_t tick = SDL_GetPerformanceCounter();
		if (tick < fps_ticks + last_swap)
			SDL_Delay((1000*(fps_ticks + last_swap - tick))/SDL_GetPerformanceFrequency());
		last_swap = SDL_GetPerformanceCounter();
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(gl_context);
	SDL_DestroyWindow(window);
	SDL_Quit();

	do_save();

	return 0;
}

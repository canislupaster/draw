#include <iostream>
#include <algorithm>
#include <vector>

#include "imgui.h"
#include <stdio.h>
#include <SDL.h>
#include <SDL_opengl.h>
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

#define NFD_THROWS_EXCEPTIONS
#include "nfd.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "point.hpp"
#include "controller.hpp"
#include "path.hpp"

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

// Simple helper function to load an image into a OpenGL texture with common settings
Texture load_image(string const& filename) {
	int image_width = 0;
	int image_height = 0;
	unsigned char* image_data = stbi_load(filename.c_str(), &image_width, &image_height, nullptr, 4);
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

int main(int argc, char** argv) {
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
	SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window* window = SDL_CreateWindow("Plotter", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
	if (window == nullptr) sdlerr();

	SDL_GLContext gl_context = SDL_GL_CreateContext(window);
	SDL_GL_MakeCurrent(window, gl_context);
	SDL_GL_SetSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

	ImGui::StyleColorsDark();
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

	ifstream fin("./last_path.txt");
	if (string last_path; fin.is_open() && getline(fin, last_path))
		plot.emplace(last_path);

	float slider_time=1.0, slider_overlay_scale=0.5, slider_path_scale=0.5, zoom=1.0;
	Pt pan_center = dim/2;

	auto draw_image = load_image("./draw.png"), move_image = load_image("./move.png");

	bool err_open=false;

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

		ImGui::Begin("Settings");

		ImGui::SeparatorText("Preview options");
		ImGui::BeginGroup();
			ImGui::SliderFloat("Time", &slider_time, 0.0, 1.0);
			ImGui::SliderFloat("Scale overlay", &slider_overlay_scale, 0.0, 1.0);
			ImGui::SliderFloat("Scale path", &slider_path_scale, 0.0, 1.0);
			ImGui::SliderFloat("Zoom", &zoom, 1.0, 15.0);

			if (ImGui::Button("Reset position")) {
				pan_center = dim/2;
			}
		ImGui::EndGroup();

		ImGui::SeparatorText("Control");

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
			if (cur_control) {
				auto& g = state.emplace(std::move(cur_control->guard()));
				if (g->err) err=g->err->what();

				if (g->running || g.paused() || g->cancelled) {
					if (g->cancelled) {
						ImGui::Text("Cancelling...");
					} else {
						ImGui::Text(g.paused() ? "PAUSED" : "DRAWING");
						if (ImGui::Button(g.paused() ? "Resume" : "Stop")) {
							if (g.paused()) g.resume(); else g.pause();
						}
					}
				} else if (ImGui::Button("Start")) {
					Controller::start(cur_control, plot->paths);
				}

				ImGui::SameLine();
				if (ImGui::Button("Cancel")) g.cancel(false);
				ImGui::SameLine();
				if (ImGui::Button("Reset")) g.cancel(true);

				if (!g->running && ImGui::Button("Disconnect")) {
					cur_control.reset();
				}
			} else if (plot && ImGui::Button("Connect")) {
				cur_control = make_shared<Controller>();
			}

			if ((!state || (!state->get().running && !state->paused()))
				&& ImGui::Button("Load path")) {

				NFD::UniquePath outPath;
				nfdfilteritem_t filterItem[1] = {{"SVG", "svg"}};

				if (NFD::OpenDialog(outPath, filterItem, 1)==NFD_OKAY) {
					plot.emplace(std::string(outPath.get()));

					ofstream fout("./last_path.txt");
					fout<<outPath.get()<<endl;
				}
			}
		} catch (exception const& ex) {
			err = ex.what();
		}

		ImGui::End();

		// ImGui::SetNextWindowSizeConstraints(ImVec2(1,1), ImGui::GetMainViewport()->Size, size_cb);
		if (wind_pos) ImGui::SetNextWindowPos(*wind_pos);
		ImGui::Begin("Preview", nullptr, ImGuiWindowFlags_None);

		if (plot) {
			auto cur_sz = ImGui::GetWindowSize();
			float plot_bound_sc = min(cur_sz.x/dim.x, cur_sz.y/dim.y);

			auto drawlist = ImGui::GetWindowDrawList();
			auto windowPosIm = ImGui::GetWindowPos();
			Pt windowPos(windowPosIm);

			auto affine_trans = windowPos + (dim/2 - pan_center*zoom)*plot_bound_sc;
			float trans_mul = plot_bound_sc*zoom;

			auto trans = [windowPos, trans_mul, affine_trans](Pt x) {
				return (x*trans_mul + affine_trans).imvec();
			};

			auto render_cmd = [&drawlist, slider_path_scale, trans, &palette](DrawCmd const& c, int pi) {
				if (c.ty==DrawCmd::Cubic)
					drawlist->AddBezierCubic(trans(c.pts[0]), trans(c.pts[1]),
						trans(c.pts[2]), trans(c.pts[3]),
						palette[pi%palette.size()], 5.0f*slider_path_scale);
				else
					drawlist->AddLine(trans(c.pts[0]), trans(c.pts[1]),
						palette[pi%palette.size()], 5.0f*slider_path_scale);
			};

			drawlist->AddRect(trans(Pt(0,0)), trans(dim), IM_COL32(255,255,0,255), 0, 0, 3.0*slider_overlay_scale);

			ImGui::InvisibleButton("previewBg", ImGui::GetContentRegionAvail());
			if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && ImGui::IsItemActive()) {
				wind_pos = windowPosIm;

				Pt del = ImGui::GetMouseDragDelta();
				pan_center = pan_center - del/trans_mul;
				ImGui::ResetMouseDragDelta();
			} else {
				wind_pos.reset();
			}

			Pt last;
			auto ft = plot->find_t(slider_time*plot->total_time);

			for (int pi=0; pi<min((int)plot->paths.size(), ft.path_i+1); pi++) {
				auto const& c = plot->paths[pi].cmds;
				int max_cmdi = ft.path_i==pi ? ft.cmd_i : c.size();
				for (int i=0; i<max_cmdi; i++) {
					render_cmd(c[i], pi);
				}

				if (max_cmdi!=-1 && max_cmdi<c.size()-1) {
					auto np = c[max_cmdi].trim(0, ft.cmd_t);
					render_cmd(np, pi);
				}

				for (int i=0; i<max_cmdi; i++) {
					drawlist->AddCircleFilled(trans(c[i].start()),
						8.0*slider_overlay_scale, IM_COL32(255,255,0,255));
					if (i==max_cmdi-1)
						drawlist->AddCircleFilled(trans(c[i].end()),
							8.0*slider_overlay_scale, IM_COL32(255,255,0,255));
				}

				if (pi>0) {
					const float dash_len = maxw/70.0/zoom;
					Pt dif = plot->paths[pi].start() - last;
					float tot = dif.norm(), inc=dash_len/tot;

					Pt prev = last; bool d=true;
					for (float t=0.0; t<1.0; t+=inc, d=!d) {
						Pt nxt = last + dif*min(t+inc, 1.0f);
						if (d || t+inc>=1.0)
							drawlist->AddLine(trans(prev), trans(nxt), IM_COL32_WHITE, 4.0f*slider_path_scale);
						prev=nxt;
					}
				}

				last = plot->paths[pi].end();
			}

			drawlist->AddCircleFilled(trans(ft.pt), 12.0*slider_overlay_scale, IM_COL32(0, 255, 255, 255));

			if (state && state->get().last_stat) {
				auto [current, from, to, is_drawing] = *state->get().last_stat;
				const float tri_w = 10.0*slider_overlay_scale/plot_bound_sc;

				Pt dif = to - from, perp = Pt(-dif.y, dif.x).unit();
				Pt up_cur = current+dif.unit()*tri_w*2;
				Pt l = current - perp*tri_w, r = current+perp*tri_w,
					ul = up_cur - perp*tri_w, ur = up_cur+perp*tri_w;

				drawlist->AddImageQuad(is_drawing ? draw_image.texture : move_image.texture,
					trans(l), trans(r), trans(ur), trans(ul),
					ImVec2(1,0), ImVec2(1,1), ImVec2(0,1), ImVec2(0,0),
					IM_COL32(255, 0, 0, 255));

				drawlist->AddLine(trans(from), trans(to), IM_COL32(255, 0, 0, 255), 6.0f*slider_overlay_scale);
			}
		} else {
			ImGui::Text("Load an SVG to see preview.");
		}

		ImGui::End();

		ImGui::Render();
		glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
		glClearColor(0.0,0.0,0.0,1.0);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		SDL_GL_SwapWindow(window);
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(gl_context);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}

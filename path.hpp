#ifndef PLOT_IMPLEMENTATION
#define PLOT_IMPLEMENTATION

#include <optional>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

#include "point.hpp"
#include <iostream>
#include <algorithm>
#include <span>

#include <string>
#include <string_view>
#include <vector>

using namespace std;

Pt eval(span<const Pt, 4> p, float t) {
	float o = 1.0-t, o2=o*o, t2=t*t;
	return p[0]*o2*o + (p[1]*o2*t + p[2]*t2*o)*3 + p[3]*t2*t;
}

Pt eval_d(span<const Pt, 4> p, float t) {
	float o = 1.0-t, o2=o*o, t2=t*t;
	return p[0]*(-3*o2) + p[1]*(3*(o-2*t)*o) + p[2]*(3*(2*o - t)*t) + p[3]*(3*t2);
}

optional<array<Pt,2>> to_line(span<const Pt, 4> curve) {
	auto d1 = curve[1]-curve[0], d2=curve[3]-curve[2];
	
}

void interpolate(span<Pt> curve, float t1=1.0f/3, float t2=2.0f/3) {
	float t12 = t1*t1, t22=t2*t2;
	float ot1 = 1-t1, ot2=1-t2, ot12=ot1*ot1, ot22=ot2*ot2;
	Pt r = curve[0]*(ot12*ot1) + curve[3]*(t12*t1);
	Pt s = curve[0]*(ot22*ot2) + curve[3]*(t22*t2);
	Pt c1 = (curve[1]-r)/3, c2=(curve[2]-s)/3;
	float a=ot12*t1, b=ot1*t12, c=ot22*t2, d=ot2*t22;
	float det = a*d-b*c;
	if (abs(det)<1e-7) throw runtime_error("can't fit to curve, bad spacing");
	//i know this is kinda bad but its better than allocating more BS
	curve[1] = (c1*d - c2*b)/det;
	curve[2] = (c2*a - c1*c)/det;
}

array<Pt, 4> trim(span<const Pt,4> curve, float begin, float end) {
	float l = end-begin;
	auto a = eval(curve, begin), b=eval_d(curve, begin)*l;
	auto c=eval_d(curve, end)*l, d=eval(curve, end);
	return {
		a, b/3+a, d-c/3, d
	};
}

void subdivide(vector<Pt>& out, span<const Pt, 4> curve, Pt lp, Pt rp, float l, float r, float threshold) {
	float m = (l+r)/2;
	Pt mid = eval(curve, m);
	if ((mid-(lp+rp)/2).sq()>threshold) {
		subdivide(out, curve, lp, mid, l, m, threshold);
		out.push_back(mid);
		subdivide(out, curve, mid, rp, m, r, threshold);
	} else {
		out.push_back(mid);
	}
}

struct Poly {
	vector<Pt> pts;
	bool closed;
	float area;

	Poly decimate(float sin_angle) {
		Poly out=*this;
		out.pts = {};

		for (int i=0; i<=pts.size()-!closed; i++) {
			Pt np = i==pts.size() ? pts[0] : pts[i];
			while (out.pts.size()>=2) {
				Pt d1 = out.pts[out.pts.size()-2]-out.pts.back(), d2=np-out.pts.back();
				if (abs(d1.det(d2)/(d1.norm()*d2.norm()))>sin_angle || d1.dot(d2)>0) break;
				out.pts.pop_back();
			}

			if (i<pts.size()) out.pts.push_back(np);
		}

		if (closed && out.pts.size()>2) {
			Pt d1 = out.pts.back()-out.pts.front(), d2=out.pts[1]-out.pts.front();
			if (abs(d1.det(d2)/(d1.norm()*d2.norm()))<=sin_angle && d1.dot(d2)<=0)
				out.pts.erase(out.pts.begin());
		}

		return out;
	}

	vector<Pt> curve(float roundness=0.0) {
		roundness /= 3;

		vector<Pt> tangents(pts.size());
		if (!closed) tangents[0] = Pt(0,0);
		for (int i=!closed; i<pts.size(); i++) {
			Pt from = pts[i==0 ? pts.size()-1 : i-1], to = pts[i];
			Pt nxt=pts[i==pts.size()-1 ? 0 : i+1];
			Pt mean = (from+nxt)/2 - to;
			tangents[i] = Pt(mean.y, -mean.x)*roundness;
			if (tangents[i].dot(to-from)<0) tangents[i]=-tangents[i];
		}

		vector<Pt> curve={closed ? pts.back() : pts.front()};
		for (int i=!closed; i<pts.size(); i++) {
			Pt from = pts[i==0 ? pts.size()-1 : i-1], to = pts[i];
			Pt t_to = tangents[i], t_from = tangents[i==0 ? pts.size()-1 : i-1];
			curve.insert(curve.end(), { from+t_from, to-t_to, to });
		}

		return curve;
	}


};

static Poly to_poly(span<const Pt> curve, float res=0.1, float threshold=5.0, bool closed=true) {
	vector<Pt> points;
	for (int i=0; i<curve.size()-1; i+=3) {
		auto c = curve.subspan(i).first<4>();
		points.push_back(c[0]);

		for (float x=0,d=1.0; x<1; x+=res) {
			float r = min(x+res, 1.0f);
			Pt l=eval(c,x), rp=eval(c, r);
			subdivide(points, c, l, rp, x,r, threshold);
		}
	}

	float area = 0.0;
	if (closed) {
		while (points.size() && (points.front()-points.back()).sq()<1e-4) points.pop_back();

		for (int i=0; i<points.size(); i++) {
			Pt nxt = points[i==points.size()-1 ? 0 : i+1];
			area += points[i].det(nxt)/2;
		}

		if (area<0) {
			area=-area;
			reverse(points.begin(), points.end());
		}
	}

	if (points.size()<3-!closed) throw runtime_error("degenerate curve");

	return { .pts=points, .closed=closed, .area=area };
}

struct DrawCmd {
	enum {
		Line, Cubic
	} ty;

	array<Pt,4> pts;
	Pt start() const {return pts[0];}
	Pt end() const {return pts[ty==Line ? 1 : 3];}
	Pt operator[](int idx) const {return pts[idx];}

	Pt eval(float t) const {
		if (ty==Cubic) return ::eval(pts, t);
		else return pts[0]*(1-t) + pts[1]*t;
	}

	DrawCmd trim(float begin, float end) const {
		if (ty==Cubic) return DrawCmd {.ty=Cubic, .pts=::trim(pts, begin, end)};
		else return DrawCmd {.ty=Line, .pts={eval(begin), eval(end)}};
	}
};

struct Path {
	vector<DrawCmd> cmds;
	vector<double> t={0};

	Pt start() const {return cmds.front().start();}
	Pt end() const {return cmds.back().start();}

	void compute_times() {
		const double dt = 0.5;

		for (int j=0; j<cmds.size(); j++) {
			if (cmds[j].ty==DrawCmd::Cubic) {
				float maxlen = 0.0;
				for (int k=0; k<3; k++)
					maxlen+=(cmds[j].pts[k]-cmds[j].pts[k+1]).norm();

				double ds = 1.0/ceil(maxlen/dt);
				double len=0;
				for (float k=0; k<1.0; k+=ds) {
					float nt = min(1.0, k+ds);
					len += (eval(cmds[j].pts, k)-eval(cmds[j].pts, nt)).norm();
				}

				t.push_back(len);
			} else {
				t.push_back((cmds[j].pts[1]-cmds[j].pts[0]).norm());
			}
		}

		for (int j=1; j<=cmds.size(); j++) t[j]+=t[j-1];
	}
};

struct Plot {
	const float move_t_mul = 0.7;

	vector<Path> paths;
	vector<double> path_t;
	float total_time;

	Plot(string const& file_path) {
		NSVGimage* img = nsvgParseFromFile(file_path.c_str(), "px", 96);

		float scale = min(maxw/img->width, maxh/img->height);
		cout<<img->width*scale/100<<" mm x "<<img->height*scale/100<<" mm"<<endl;

		vector<vector<Pt>> cubics;

		for (auto shape = img->shapes; shape != nullptr; shape = shape->next) {
			for (auto path = shape->paths; path != nullptr; path = path->next) {
				if (path->npts<4) continue;
				if ((path->npts-1)%3 != 0)
					throw runtime_error("not cubic bezier");

				auto& p = cubics.emplace_back();
				for (int i = 0; i < path->npts; i++) {
					p.emplace_back(Pt { path->pts[i*2], path->pts[i*2+1] }*scale);
				}

				if (shape->fill.type==NSVG_PAINT_COLOR && shape->opacity>0) {

				}

				// auto poly = to_poly(p);
				// poly = poly.decimate(0.01);
				// p = poly.curve(0.5);

				if (shape->strokeWidth==0.0) cubics.pop_back();
			}
		}

		nsvgDelete(img);

		if (cubics.empty()) throw runtime_error("no paths");

		struct E {int a, b; double cost;};
		vector<E> edges;

		for (int i=0; i<cubics.size(); i++) {
			for (int j=0; j<cubics.size(); j++) {
				auto a = cubics[j].front(), b = cubics[i].back();
				edges.emplace_back(E {i,j,(a-b).norm()});
			}
		}

		sort(edges.begin(), edges.end(), [](E const& a, E const& b) {
			return a.cost<b.cost;
		});

		vector<int> out(cubics.size(), -1);
		vector<int> in(cubics.size(), -1);
		vector<int> root(cubics.size());
		vector<int> rank(cubics.size(), 1);
		for (int i=0; i<cubics.size(); i++) root[i]=i;

		auto get = [&](int x) {
			while (x!=root[x]) x=root[x];
			return x;
		};

		for (E edge: edges) {
			if (out[edge.a]!=-1 || in[edge.b]!=-1) continue;
			int r1=get(edge.a), r2=get(edge.b);
			if (r1==r2) continue;

			if (rank[r1]>rank[r2]) root[r2] = r1, rank[r1]+=rank[r2];
			else root[r1] = r2, rank[r2]+=rank[r1];

			out[edge.a]=edge.b, in[edge.b]=edge.a;
		}

		int beg=-1, end=-1;
		for (int i=0; i<cubics.size(); i++) {
			if ((out[i]==-1 && end!=-1) || (in[i]==-1 && beg!=-1)) {
				throw runtime_error("bad path");
			}

			if (out[i]==-1) end=i;
			if (in[i]==-1) beg=i;
		}

		for (int i=beg; i!=-1; i=out[i]) {
			auto& p = paths.emplace_back();

			for (int i=0; i<cubics[i].size()-1; i+=3) {
				auto& c = p.cmds.emplace_back(DrawCmd {.ty=DrawCmd::Cubic });
				copy_n(cubics[i].begin()+i, 4, c.pts.begin());
			}
		}

		for (Path& p: paths) p.compute_times();
		compute_times();

		cout<<"loaded"<<endl;
	}

	void compute_times() {
		path_t.assign(paths.size(),0);

		for (int i=1; i<paths.size(); i++) {
			path_t[i] = path_t[i-1] + paths[i-1].t.back()
				+ (paths[i].start()-paths[i-1].end()).norm()*move_t_mul;
		}

		total_time = path_t.back()+paths.back().t.back();
	}

	struct FindT {
		Pt pt;
		int path_i;
		int cmd_i;
		float cmd_t;
	};

	FindT find_t(float t) {
		auto mk = [](Pt pt, int pi) {return FindT {.pt=pt, .path_i=pi, .cmd_i=-1};};
		if (t>=total_time) return mk(paths.back().end(), paths.size());
		else if (t<=0) return mk(paths.front().start(), 0);

		int idx = (upper_bound(path_t.begin(), path_t.end(), t)-path_t.begin())-1;
		t -= path_t[idx];
		auto& pt = paths[idx].t;
		int bidx = upper_bound(pt.begin(), pt.end(), t)-pt.begin();

		if (bidx==pt.size()) {
			t -= pt.back();
			Pt a=paths[idx].end(), b=paths[idx+1].start();
			return mk(a + (b-a).unit()*t/move_t_mul, idx+1);
		}

		bidx--;

		float nt = (t-pt[bidx])/(pt[bidx+1] - pt[bidx]);
		return FindT {
			.pt=paths[idx].cmds[bidx].eval(nt),
			.path_i=idx,
			.cmd_i = bidx, .cmd_t = nt
		};
	};
};

#endif
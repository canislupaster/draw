#ifndef POINT_IMPLEMENTATION
#define POINT_IMPLEMENTATION
#include <cmath>
#include <tuple>
#include <imgui.h>

const float maxw=298.75*100, maxh=244.21*100;

struct Pt {
	float x, y;
	Pt(float x=0, float y=0) : x(x), y(y) {}
	Pt(ImVec2 imvec): Pt(imvec.x, imvec.y) {}

	bool operator< (Pt Pt) const { return std::tie(x, y) < std::tie(Pt.x, Pt.y) ;}
	bool operator==(Pt Pt) const {
		return abs(x-Pt.x) < 1e-9 && abs(y-Pt.y) < 1e-9;
	}
	Pt operator+(Pt p) const {return Pt(x+p.x, y+p.y);}
	Pt operator-(Pt p) const {return Pt(x-p.x, y-p.y);}
	Pt operator-() const {return Pt(-x,-y);}
	Pt operator* (float d) const {return Pt(x*d, y*d);}
	Pt operator/(float d) const { return Pt(x/d, y/d);}
	float norm() const {return hypot(x,y);}
	float sq() const {return x*x + y*y;}
	float det(Pt const& o) const { return x*o.y - y*o.x; }
	Pt unit() const { return *this/norm(); }
	Pt mix(Pt const& other, float a) const {return Pt(a*x + (1-a)*other.x, a*y + (1-a)*other.y);}
	float dot(Pt const& o) const {return o.x*x + o.y*y;}

	ImVec2 imvec() const {return ImVec2(x,y);}
};

const Pt dim(maxw, maxh);
#endif
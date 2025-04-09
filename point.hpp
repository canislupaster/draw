#ifndef POINT_IMPLEMENTATION
#define POINT_IMPLEMENTATION
#include <cmath>
#include <ostream>
#include <istream>
#include <tuple>
#include <imgui.h>

#include "cereal/cereal.hpp"

const float maxw=298.75*100, maxh=244.21*100;

const float Epsilon = 1e-7;
const float REAL_EPS = 0.5001; //sorry, but all points are rounded to nearest integer anyways, so it doesn't really matter in some cases (for which i use this)....

struct Pt {
	float x, y;
	Pt(float x=0, float y=0) : x(x), y(y) {}
	Pt(ImVec2 imvec): Pt(imvec.x, imvec.y) {}

	bool operator<(Pt Pt) const { return std::tie(y, x) < std::tie(Pt.y, Pt.x); }
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
	Pt unit() const {
		float den = norm();
		return den<Epsilon ? Pt(0,0) : *this/den;
	}
	Pt mix(Pt const& other, float a) const {return Pt(a*x + (1-a)*other.x, a*y + (1-a)*other.y);}
	float dot(Pt const& o) const {return o.x*x + o.y*y;}
	Pt perp(bool ccw=true) const {return Pt(ccw ? -y : y, ccw ? x : -x);}

	ImVec2 imvec() const {return ImVec2(x,y);}

	friend std::ostream& operator<<(std::ostream& os, Pt const& p) {
		return os << "(" << p.x << "," << p.y << ")";
	}

	friend std::istream& operator>>(std::istream& is, Pt& p) {
		is.ignore(1, '(');
		is>>p.x;
		is.ignore(1, ',');
		is>>p.y;
		is.ignore(1, ')');

		return is;
	}

	template<class Archive>
	void serialize(Archive& archive) {
		archive(CEREAL_NVP(x), CEREAL_NVP(y));
	}
};

const Pt dim(maxw, maxh);
#endif
#ifndef PLOT_IMPLEMENTATION
#define PLOT_IMPLEMENTATION

#include <limits>
#include <list>
#include <map>
#include <set>
#include <optional>
#include <queue>
#include <random>
#include <stack>
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

struct UnionFind {
	vector<int> root;
	vector<int> rank;
	UnionFind(int n): root(n), rank(n,1) {
		for (int i=0; i<n; i++) root[i]=i;
	}

	int get(int x) {
		while (x!=root[x]) x=root[x];
		return x;
	};

	bool join(int a, int b) {
		int r1=get(a), r2=get(b);
		if (r1==r2) return false;

		if (rank[r1]>rank[r2]) root[r2] = r1, rank[r1]+=rank[r2];
		else root[r1] = r2, rank[r2]+=rank[r1];
		return true;
	}
};

struct KDTree {
	enum {X,Y,Leaf,Empty} ty;
	Pt a,b;
	int leaf_i;

	unique_ptr<KDTree> l,r;
	KDTree* parent;

	const float fmax = numeric_limits<float>::max(), fmin = numeric_limits<float>::min();

	KDTree(span<const Pt> pts, span<int> pts_ix, span<int> pts_iy, KDTree* parent=nullptr): parent(parent) {
		int n = pts_ix.size();
		if (n==0) {
			a={fmax,fmax}, b={fmin,fmin};
			ty=Empty;
			return;
		}

		a=b=pts[pts_ix[0]];
		for (int i=1; i<n; i++) {
			int j = pts_ix[i];
			a.x = min(a.x, pts[j].x), a.y = min(a.y, pts[j].y);
			b.x = max(b.x, pts[j].x), b.y = max(b.y, pts[j].y);
		}

		// oh god
		float xmid = pts[pts_ix[n/2]].x, ymid=pts[pts_iy[n/2]].y;
		int b4x = lower_bound(pts_ix.begin(), pts_ix.end(), xmid, [&pts](int i, float x) {return pts[i].x<x;})-pts_ix.begin();
		int b4y = lower_bound(pts_iy.begin(), pts_iy.end(), ymid, [&pts](int i, float y) {return pts[i].y<y;})-pts_iy.begin();

		if ((b4x==0 && b4y==0) || (b4x==n && b4y==n)) {
			//all points are equal, treat as leaf
			ty=Leaf, leaf_i=pts_ix[0];
			return;
		}

		//choose most balanced axis
		int k;
		if (abs(2*b4x-n) < abs(2*b4y-n)) {
			ty=X, k=b4x;
			stable_partition(pts_iy.begin(), pts_iy.end(), [xmid, &pts](int i) {return pts[i].x<xmid;});
		} else {
			ty=Y, k=b4y;
			stable_partition(pts_ix.begin(), pts_ix.end(), [ymid, &pts](int i) {return pts[i].y<ymid;});
		}

		l = make_unique<KDTree>(pts, span(pts_ix).first(k), span(pts_iy).first(k), this);
		r = make_unique<KDTree>(pts, span(pts_ix).last(n-k), span(pts_iy).last(n-k), this);
	}

	static KDTree make(span<const Pt> pts) {
		vector<int> pts_ix(pts.size()), pts_iy(pts.size());
		iota(pts_ix.begin(), pts_ix.end(), 0);
		sort(pts_ix.begin(), pts_ix.end(), [&pts](int a, int b) {return pts[a].x<pts[b].x;});

		iota(pts_iy.begin(), pts_iy.end(), 0);
		sort(pts_iy.begin(), pts_iy.end(), [&pts](int a, int b) {return pts[a].y<pts[b].y;});

		return {pts, span(pts_ix), span(pts_iy)};
	}

	void remove() {
		ty=Empty;
		a={fmax,fmax}, b={fmin,fmin};
		for (KDTree* cur = parent; cur; cur=cur->parent) {
			// so stupid
			cur->a.x = min(cur->l->a.x, cur->r->a.x);
			cur->a.y = min(cur->l->a.y, cur->r->a.y);
			cur->b.x = max(cur->l->b.x, cur->r->b.x);
			cur->b.y = max(cur->l->b.y, cur->r->b.y);

			if (cur->l->ty==Empty && cur->r->ty==Empty)
				cur->ty=Empty;
		}
	}
	
	float dist(Pt p) {
		if (ty==Leaf) return (a-p).norm();
		else if (ty==Empty) return numeric_limits<float>::max();

		Pt closest(clamp(p.x, a.x, b.x), clamp(p.y, a.y, b.y));
		return (closest-p).norm();
	}

	struct Nearest {
		struct State {
			KDTree* node;
			float dist;
			bool operator<(State const& o) const {return dist>o.dist;}
		};

		priority_queue<State> q;
		Pt p;

		KDTree* next() {
			KDTree* best=nullptr;
			float cur_dist = numeric_limits<float>::max();
			
			while (q.size()) {
				auto [cur, ndist] = q.top();

				if (ndist>=cur_dist) break;
				else q.pop();

				if (cur->ty==Leaf) {
					if (cur->a == p) continue;
					best=cur, cur_dist=ndist;
					continue;
				} else if (cur->ty==Empty) {
					continue;
				}

				q.push(State {.node=cur->l.get(), .dist=cur->l->dist(p)});
				q.push(State {.node=cur->r.get(), .dist=cur->r->dist(p)});
			}
			
			return best;
		}
	};

	Nearest nearest(Pt p) {
		priority_queue<Nearest::State> q;
		q.push(Nearest::State {.node=this, .dist=dist(p)});
		return {q,p};
	}
};

Pt eval(span<const Pt, 4> p, float t) {
	float o = 1.0-t, o2=o*o, t2=t*t;
	return p[0]*o2*o + (p[1]*o2*t + p[2]*t2*o)*3 + p[3]*t2*t;
}

Pt eval_d(span<const Pt, 4> p, float t) {
	float o = 1.0-t, o2=o*o, t2=t*t;
	return p[0]*(-3*o2) + p[1]*(3*(o-2*t)*o) + p[2]*(3*(2*o - t)*t) + p[3]*(3*t2);
}

optional<array<Pt,2>> to_line(span<const Pt, 4> curve) {
	auto d1 = curve[3]-curve[0], d2=curve[2]-curve[0], d3=curve[1]-curve[0];
	if (abs(d1.det(d2))>REAL_EPS || abs(d1.det(d3))>REAL_EPS)
		return nullopt;

	float b=d3.dot(d1), c=d2.dot(d1), d=d1.sq();
	if (b<0 || c<0 || b>d || c>d) return nullopt;

	//oof im so dum
	// float x = 2*c-4*b, y=3*(b-c) + d;
	// float der = x*x - 4*y*b;

	// float tmin=0, tmin_v=0, tmax=1, tmax_v=d;
	// auto chk = [b,c,d,&tmin,&tmax,&tmin_v,&tmax_v](float t) {
	// 	if (t<0 || t>1) return;

	// 	float ot = 1-t;
	// 	float v=t*(3*(b*ot*ot + c*ot*t) + d*t*t);
	// 	if (v<tmin) tmin=t, tmin_v=v;
	// 	else if (v>tmax) tmax=t, tmax_v=v;
	// };

	// bool y_nonzero = abs(y)>EPS;
	// if (y_nonzero && der>=0) {
	// 	float v = sqrt(der);
	// 	chk((-x+v)/(2*y));
	// 	if (v>EPS) chk((-x-v)/(2*y));
	// } else if (!y_nonzero && abs(x)>EPS) {
	// 	chk(-b/x);
	// }

	return {{curve[0], curve[3]}};
}

void interpolate(span<Pt> curve, float t1=1.0f/3, float t2=2.0f/3) {
	float t12 = t1*t1, t22=t2*t2;
	float ot1 = 1-t1, ot2=1-t2, ot12=ot1*ot1, ot22=ot2*ot2;
	Pt r = curve[0]*(ot12*ot1) + curve[3]*(t12*t1);
	Pt s = curve[0]*(ot22*ot2) + curve[3]*(t22*t2);
	Pt c1 = (curve[1]-r)/3, c2=(curve[2]-s)/3;
	float a=ot12*t1, b=ot1*t12, c=ot22*t2, d=ot2*t22;
	float det = a*d-b*c;
	if (abs(det)<Epsilon) throw runtime_error("can't fit to curve, bad spacing");
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

		for (int i=0; i+!closed<=pts.size(); i++) {
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

	struct Tri {
		Pt a,b,c;
		float area() const { return abs((c-a).det(b-a))/2; }
		Pt sample(float u, float v) const {
			return a + (b-a)*u + (c-a)*v;
		}
	};

	struct Line {
		Pt a,b;

		optional<float> intersect(Line const& y) const {
			Pt d1 = y.a-a, d2=y.b-a, d3=b-a;
			float det1=d1.det(d3), det2=d2.det(d3), d3sq = d3.sq();

			if (abs(det1-det2)<=Epsilon || d3sq<Epsilon) {
				// if (abs(det1)>EPS) return false;
				//lines are colinear
				// float dot1 = d1.dot(d3), dot2=d2.dot(d3);
				// return !((dot1<0 && dot2<0) || (dot1>d3sq && dot2>d3sq));
				return nullopt;
			}

			float t = -det1/(det2-det1);
			if (t<0 || t>1) return nullopt;

			Pt intersection = y.a + (y.b-y.a)*t;
			float d = (intersection-a).dot(d3)/d3sq;
			return d>=0 && d<=1 ? optional(d) : nullopt;
		}
	};

	bool intersect(Line const& l) const {
		for (int i=0; i<pts.size(); i++) {
			Line edge = {pts[i], pts[i==pts.size()-1 ? 0 : i+1]};
			if (edge.intersect(l)) return true;
		}

		return false;
	}

	bool towards_inside(int i, Pt outwards) const {
		Pt l = pts[i==0 ? pts.size()-1 : i-1]-pts[i], r=pts[i==pts.size()-1 ? 0 : i+1]-pts[i];
		float det = l.det(r);
		return (det>0 && !(r.det(outwards)<0 && l.det(outwards)>0))
			|| (det<0 && r.det(outwards)>=0 && l.det(outwards)<=0);
	}

	//i+2 <= j
	vector<vector<bool>> compute_inside() const {
		vector<vector<bool>> inside_poly(pts.size(), vector<bool>(pts.size(), false));

		for (int i=0; i<pts.size(); i++) {
			if (i<pts.size()-1) inside_poly[i][i+1]=true;
			else inside_poly[i][0]=true;

			for (int j=i+2; j<pts.size(); j++) {
				Pt d = pts[j]-pts[i];
				if (towards_inside(i,d)) {
					Line line = {pts[i], pts[j]};

					inside_poly[i][j]=true;
					for (int k=0; k<pts.size(); k++) {
						int nxt = k==pts.size()-1 ? 0 : k+1;
						if (i==k || i==nxt || k==j || nxt==j) continue;

						auto intersect = Line{pts[k], pts[nxt]}.intersect(line);
						if (intersect) {
							if (*intersect<Epsilon && towards_inside(i, d)) continue;
							if (*intersect>1-Epsilon && towards_inside(nxt, d)) continue;

							inside_poly[i][j]=false;
							break;
						}
					}
				}
			}
		}

		return inside_poly;
	}

	//two-ears
	vector<Tri> triangulate() const {
		if (!closed) throw runtime_error("can't triangulate open polygon");

		auto inside = compute_inside();

		list<int> idx;
		for (int i=0; i<pts.size(); i++) idx.push_back(i);
		// 	cout<<"("<<pts[i].x<<","<<pts[i].y<<"),";
		// }
		// cout<<endl;

		vector<Tri> out;
	
		for (int sz=idx.size(); idx.size()>=3; sz=idx.size()) {
			for (auto it=idx.begin(); it!=idx.end();) {
				int p = it==idx.begin() ? idx.back() : *prev(it);
				int n = next(it)==idx.end() ? idx.front() : *next(it);
				bool degen = abs((pts[p]-pts[*it]).det(pts[n]-pts[*it]))<EPS;
				if (degen || ((inside[p][n] || inside[n][p])
						&& (inside[p][*it] || inside[*it][p])
						&& (inside[n][*it] || inside[*it][n]))) {
					out.emplace_back(Tri {pts[p], pts[*it], pts[n]});
					it = idx.erase(it);
				} else {
					it++;
				}
			}

			if (idx.size()>=sz) throw runtime_error("can't triangulate");
		}

		return out;
	}

	// poly is counterclockwise, closed (last pt != first pt) in rect up to maxw,maxh exclusive
	vector<Poly> scribble(float value) {
		auto tris = triangulate();

		map<float, int> area_tri;
		float cur=0.0;
		for (int i=0; i<tris.size(); i++) {
			cur+=tris[i].area();
			area_tri.emplace(cur, i);
		}

		//sample from total area
		mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
		uniform_real_distribution<float> dist(0.0, cur);
		uniform_real_distribution<float> dist01(0.0, 1.0);

		int npt = ceil(value*cur);
		set<Pt> samples;

		for (int i=0; i<npt; i++) {
			float x = dist(rng);
			Tri& tri = tris[area_tri.upper_bound(x)->second];

			float u = dist01(rng), v = dist01(rng);
			if (u+v>1) u=1-u, v=1-v;

			samples.insert(tri.sample(u,v));
		}

		vector<Pt> vec(samples.begin(), samples.end());
		npt=vec.size();

		vector<int> in(npt,-1), out(npt,-1);
		UnionFind uf(npt);

		KDTree tree = KDTree::make(vec);

		for (int i=0; i<npt; i++) {
			auto nn = tree.nearest(vec[i]);
			for (int j=0; j<20 && (in[i]==-1 || out[i]==-1); j++) {
				auto nxt = nn.next();
				bool cont = !nxt || (in[i]!=-1 && in[nxt->leaf_i]!=-1)
					|| (out[i]!=-1 && out[nxt->leaf_i]!=-1)
					|| intersect({vec[i], vec[nxt->leaf_i]});

				if (!cont && uf.join(i, nxt->leaf_i)) {
					if (in[i]==-1 && out[nxt->leaf_i]==-1) {
						in[i]=nxt->leaf_i, out[nxt->leaf_i]=i;
					} else if (out[i]==-1 && in[nxt->leaf_i]==-1) {
						in[nxt->leaf_i]=i, out[i]=nxt->leaf_i;
					}
				}

				if (nxt && in[nxt->leaf_i]!=-1 && out[nxt->leaf_i]!=-1)
					nxt->remove();
			}
		}

		vector<bool> visited(npt, false);
		vector<Poly> polys;

		for (int i=0; i<npt; i++) {
			if (visited[i] || in[i]!=-1) continue;
			auto& p = polys.emplace_back(std::move(Poly {.closed=false, .area=0.0f}));
			for (int j=i; out[j]!=-1; j=out[j]) {
				visited[j]=true;
				p.pts.push_back(vec[j]);
			}

			if (p.pts.size()<2) polys.pop_back();
		}

		return polys;
	}

	void cleanup() {
		vector<Pt> newpt;
		for (int i=0; i<pts.size(); i++) {
			if (newpt.empty() || (newpt.back()-pts[i]).sq()>REAL_EPS)
				newpt.push_back(pts[i]);
		}

		pts.swap(newpt);

		area = 0.0;

		if (closed) {
			if (pts.size() && (pts.front()-pts.back()).sq()<REAL_EPS) pts.pop_back();

			for (int i=0; i<pts.size(); i++) {
				Pt nxt = pts[i==pts.size()-1 ? 0 : i+1];
				area += pts[i].det(nxt)/2;
			}

			if (area<0) {
				area=-area;
				reverse(pts.begin(), pts.end());
			}
		}

		// if (pts.size()<3-!closed) throw runtime_error("degenerate curve");
	}

	static vector<Poly> monotone_fill(vector<Poly> polys, float dist);
};

//non self intersecting, simple except can have holes
vector<Poly> Poly::monotone_fill(vector<Poly> polys, float dist) {
	vector<Poly> out;
	
	struct Chain;
	struct E {
		enum {Start, Between, End} ty;
		vector<Chain*> chain;
		array<multimap<Pt, E>::iterator, 2> adj;
	};

	auto order_adj = [](multimap<Pt,E>::iterator it) {
		auto a1=it->second.adj[0], a2=it->second.adj[1];
		Pt d1=a1->first-it->first, d2=a2->first-it->first;
		if (d1.det(d2)>0) swap(a1,a2);

		return make_pair(a1,a2);
	};

	multimap<Pt, E> pts;
	for (auto& poly: polys) {
		decltype(pts.begin()) prev_it, first_it;
		for (int i=0; i<poly.pts.size(); i++) {
			auto e = pts.emplace(poly.pts[i], E {});
			if (i>0) {
				e->second.adj[0] = prev_it;
				prev_it->second.adj[1] = e;
			} else {
				first_it=e;
			}

			prev_it=e;
		}

		if (!poly.closed) throw runtime_error("monotone fill requires closed polys");

		prev_it->second.adj[1] = first_it;
		first_it->second.adj[0] = prev_it;
	}

	for (auto& [pt, e]: pts) {
		if (e.adj[1]->first<e.adj[0]->first) swap(e.adj[0], e.adj[1]);
		if (pt<e.adj[0]->first) e.ty=E::Start;
		else if (e.adj[1]->first<pt) e.ty=E::End;
		else e.ty=E::Between;
	}
	
	struct ActiveChain;
	struct Chain {
		vector<Pt> left_pts, right_pts;
		multimap<Pt,E>::iterator left, right, lto, rto;
		set<ActiveChain>::iterator active_it;

		struct Inside {float l,r;};

		Inside inside(float y) const {
			Pt lpt = left->first, rpt = right->first;
			Pt nlpt = lto->first-lpt, nrpt = rto->first-rpt;

			bool lz = abs(nlpt.y)<Epsilon, rz = abs(nrpt.y)<Epsilon;

			float l = (lz ? 0 : (y-lpt.y)/nlpt.y * nlpt.x) + lpt.x;
			float r = (rz ? 0 : (y-rpt.y)/nrpt.y * nrpt.x) + rpt.x;
			
			return Inside {l,r};
		}
		
		bool cmp_y(float y, Chain const& o) const {
			auto a = inside(y), b = o.inside(y);
			//apparently necessary, since a split will make 2 chains with same left, and then they are ordered by right... i wish i could order during insertion but then i need a real b tree
			return std::tie(a.l, a.r)<std::tie(b.l, b.r);
		}
	};

	//ill brute force for now, can replace with map (and make chain a union) later
	vector<unique_ptr<Chain>> chains;

	struct ActiveChain {
		enum {Search, Element} ty;
		union {
			Chain* c;
			Pt pt;
		};

		float y;

		bool operator<(ActiveChain const& o) const {
			if (ty==Element && o.ty==Element)
				return c->cmp_y(max(y,o.y), *o.c);

			ActiveChain const* t = this, *op = &o;
			if (o.ty==Search) swap(t,op);
			auto in = op->c->inside(t->pt.y);
			return in.r < t->pt.x;
		}
	};

	auto swap_chain = [](E& e, Chain* old, Chain* x) {
		auto it = find(e.chain.begin(), e.chain.end(), old);
		if (it==e.chain.end()) throw runtime_error("chain not found");
		*it=x;
	};

	set<ActiveChain> active;
	vector<Chain*> done;

	for (auto it=pts.begin(); it!=pts.end(); it++) {
		auto& [pt, e] = *it;

		//see if belongs in a chain
		if (e.ty==E::Between) {
			for (auto c : e.adj[0]->second.chain) {
				if (c->lto == it) {
					c->left = it, c->lto = e.adj[1];
					c->left_pts.push_back(pt);
				} else if (c->rto == it) {
					c->right = it, c->rto = e.adj[1];
					c->right_pts.push_back(pt);
				} else continue;

				e.chain.push_back(c);
				break;
			}

			if (e.chain.empty()) throw runtime_error("no chain found");

			continue;
		} else if (e.ty==E::End) { //end/merge
			auto pred = [&](Chain* c) {
				return c->rto==it || c->lto==it;
			};

			auto c1 =*find_if(e.adj[0]->second.chain.begin(), e.adj[0]->second.chain.end(), pred),
				c2 =*find_if(e.adj[1]->second.chain.begin(), e.adj[1]->second.chain.end(), pred);

			if (c1!=c2) { //merge
				if (c1->lto==it) swap(c1,c2);
				float l = c1->inside(pt.y).l;

				c1->left_pts.emplace_back(l, pt.y);

				c2->left_pts.push_back(pt);
				c2->left_pts.emplace_back(l, pt.y);
				c2->left = c1->left, c2->lto=c1->lto;
				swap_chain(c2->left->second, c1, c2);
			} else {
				c1->left_pts.push_back(pt);
			}

			c1->right_pts.push_back(pt);
			done.push_back(c1);
			active.erase(c1->active_it);
			continue;
		}

		Chain* chain=nullptr;
		float l;

		auto active_it = active.lower_bound(ActiveChain {
			.ty=ActiveChain::Search, .pt=pt, .y=pt.y
		});

		if (active_it!=active.end()) {
			auto in = active_it->c->inside(pt.y);
			if (pt.x >= in.l && pt.x <= in.r) chain=active_it->c;
			l=in.l;
		}

		//create new chain
		auto ord = order_adj(it);

		if (!chain) {
			Chain* c = chains.emplace_back(std::move(make_unique<Chain>(Chain { //new
				.left_pts = {pt}, .right_pts = {pt},
				.left=it, .right=it,
				.lto=ord.first, .rto=ord.second
			}))).get();

			c->active_it = active.insert(ActiveChain {
				.ty=ActiveChain::Element, .c=c, .y=pt.y
			}).first;

			e.chain.push_back(c);
			
		//hole -- split/merge
		} else { //split
			Pt x(l, pt.y); //split with horizontal for cleaner fill i hope

			Chain* new_chain = chains.emplace_back(std::move(make_unique<Chain>(Chain { //new
				.left_pts = {x}, .right_pts={pt},
				.left = chain->left, .right=it,
				.lto = chain->lto, .rto = ord.first
			}))).get();

			new_chain->active_it = active.insert(ActiveChain {
				.ty=ActiveChain::Element, .c=new_chain, .y=pt.y
			}).first;

			if (chain->left==chain->right) { //all these fucking edge cases
				chain->left->second.chain.push_back(new_chain);
			} else {
				swap_chain(chain->left->second, chain, new_chain);
			}

			chain->left_pts.push_back(x);
			chain->left_pts.push_back(pt);
			chain->left = it;
			chain->lto = ord.second;
			e.chain = {chain, new_chain};
		}
	}

	for (Chain* c: done) {
		Poly p {.closed=false, .area=0.0f};

		auto clean_chain = [](vector<Pt>& pts) {
			vector<Pt> new_pts;
			for (int i=0; i<pts.size(); i++) {
				while (new_pts.size()>=2 && abs((new_pts[new_pts.size()-2]-new_pts.back()).det(pts[i]-new_pts.back()))<EPS)
					new_pts.pop_back();

				if (new_pts.empty() || (new_pts.back()-pts[i]).sq()>1e-4)
					new_pts.push_back(pts[i]);
			}

			pts.swap(new_pts);
		};

		clean_chain(c->left_pts);
		clean_chain(c->right_pts);

		reverse(c->right_pts.begin(), c->right_pts.end());
		reverse(c->left_pts.begin(), c->left_pts.end());

		int sy = ceil(c->left_pts.back().y/(2*dist));
		int ny = floor<int>(c->left_pts.front().y/(2*dist))+1-sy;

		struct Ev {
			enum {Left, Right, Y} ty;
			float y;

			bool operator<(Ev const& o) const {return y>o.y;}
		};

		priority_queue<Ev> ev;
		for (int a=0; a<ny; a++) ev.push(Ev {.ty=Ev::Y, .y=(a+sy)*2.0f*dist});

		Pt sl=c->left_pts.back(), sr=c->right_pts.back();
		c->left_pts.pop_back(), c->right_pts.pop_back();

		for (Pt& x: c->left_pts) ev.push(Ev {.ty=Ev::Left, .y=x.y});
		for (Pt& x: c->right_pts) ev.push(Ev {.ty=Ev::Right, .y=x.y});

		bool is_left=false, is_right=false;
		while (ev.size()) {
			auto [ty, y] = ev.top(); ev.pop();

			switch (ty) {
				case Ev::Left:
					sl = c->left_pts.back();
					c->left_pts.pop_back();
					break;
				case Ev::Right:
					sr = c->right_pts.back();
					c->right_pts.pop_back();
					break;
				case Ev::Y: break;
			}

			if (c->left_pts.empty() || c->right_pts.empty()) break;

			Pt ls=c->left_pts.back()-sl, rs=c->right_pts.back()-sr;
			Pt o=sl + (ls.y<EPS ? 0 : ls*(y-sl.y)/ls.y),
				t=sr + (rs.y<EPS ? 0 : rs*(y-sr.y)/rs.y);

			float lx = ls.y<EPS ? numeric_limits<float>::max() : dist*max(1.0f,abs(ls.x/ls.y));
			float rx = rs.y<EPS ? numeric_limits<float>::max() : dist*max(1.0f,abs(rs.x/rs.y));
			Pt l=o+Pt(lx,0), r=t-Pt(rx,0);
			l.x = min(l.x, max(c->left_pts.back().x, sl.x)+dist);
			r.x = max(r.x, min(c->right_pts.back().x, sr.x)-dist);

			if (l.x > r.x) {
				l=r=abs(ls.x*rs.y) < EPS ? o : (o+(t-o)/(1 + abs(ls.y*rs.x/(ls.x*rs.y))));
			}

			bool diff = r.x-l.x > EPS;

			switch (ty) {
				case Ev::Left:
					if (is_left) p.pts.push_back(l);
					break;
				case Ev::Right:
					if (is_right) p.pts.push_back(r);
					break;
				case Ev::Y:
					if (!is_left) {
						p.pts.push_back(r);
						if (diff) p.pts.push_back(l);
						is_right=false, is_left=true;
					} else if (!is_right) {
						p.pts.push_back(l);
						if (diff) p.pts.push_back(r);
						is_right=true, is_left=false;
					}

					if (--ny == 0) ev={};

					break;
			}
		}

		p.cleanup();
		out.emplace_back(std::move(p));
	}

	return out;
}

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

	Poly p { .pts=points, .closed=closed };
	p.cleanup();
	return p;
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

	void rev() {
		if (ty==Cubic) reverse(pts.begin(), pts.end());
		else swap(pts[0], pts[1]);
	}
};

struct Path {
	vector<DrawCmd> cmds;
	vector<double> t;
	bool closed;

	uint32_t color;

	Pt start() const {return cmds.front().start();}
	Pt end() const {return cmds.back().end();}

	void compute_times() {
		const double dt = 0.5;
		t = {0};

		for (auto & cmd : cmds) {
			if (cmd.ty==DrawCmd::Cubic) {
				float maxlen = 0.0;
				for (int k=0; k<3; k++)
					maxlen+=(cmd.pts[k]-cmd.pts[k+1]).norm();

				double ds = 1.0/ceil(maxlen/dt);
				double len=0;
				for (float k=0; k<1.0; k+=ds) {
					float nt = min(1.0, k+ds);
					len += (eval(cmd.pts, k)-eval(cmd.pts, nt)).norm();
				}

				t.push_back(len);
			} else {
				t.push_back((cmd.pts[1]-cmd.pts[0]).norm());
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

	Plot(string const& file_path, float fill_angle, float fill_dist) {
		NSVGimage* img = nsvgParseFromFile(file_path.c_str(), "px", 96);

		float scale = min(maxw/img->width, maxh/img->height);
		cout<<img->width*scale/100<<" mm x "<<img->height*scale/100<<" mm"<<endl;

		for (auto shape = img->shapes; shape != nullptr; shape = shape->next) {
			bool fill = shape->fill.type==NSVG_PAINT_COLOR && shape->opacity>Epsilon;
			vector<Poly> to_fill;

			for (auto path = shape->paths; path != nullptr; path = path->next) {
				if (path->npts<4) continue;
				if ((path->npts-1)%3 != 0)
					throw runtime_error("not cubic bezier");

				vector<Pt> pts;
				for (int i = 0; i < path->npts; i++) {
					pts.emplace_back(Pt { path->pts[i*2], path->pts[i*2+1] }*scale);
				}

				bool closed = path->closed;
				if ((pts.back()-pts.front()).sq()<REAL_EPS) closed=true;
				
				if (fill && closed) {
					auto poly = to_poly(pts);
					poly = poly.decimate(0.01);
					to_fill.emplace_back(std::move(poly));
				}

				if (shape->strokeWidth>0.0 && shape->stroke.type==NSVG_PAINT_COLOR) {
					auto& p = paths.emplace_back(std::move(Path {
						.closed = static_cast<bool>(closed),
						.color = shape->stroke.color
					}));

					for (int j=0; j<pts.size()-1; j+=3) {
						auto l = to_line(span(pts).subspan(j).first<4>());

						if (l) {
							auto& c = p.cmds.emplace_back(DrawCmd {
								.ty=DrawCmd::Line,
								.pts={ l->at(0), l->at(1) }
							});
						} else {
							auto& c = p.cmds.emplace_back(DrawCmd {.ty=DrawCmd::Cubic });
							copy_n(pts.begin()+j, 4, c.pts.begin());
						}
					}
				}
			}

			if (fill) {
				float c = sin(fill_angle), s = cos(fill_angle);
				for (auto& poly: to_fill) {
					for (auto& pt: poly.pts) {
						pt = Pt(pt.x*c - pt.y*s, pt.x*s + pt.y*c);
					}
				}

				auto filled = Poly::monotone_fill(to_fill, min(fill_dist/shape->opacity, 400.0f));

				for (auto& poly: filled) {
					for (auto& pt: poly.pts) {
						pt = Pt(pt.x*c + pt.y*s,  pt.y*c - pt.x*s);
					}

					poly = poly.decimate(0.05);
					if (poly.pts.size()<2) continue;

					auto& p = paths.emplace_back(std::move(Path {
						.closed=false, .color=shape->fill.color
					}));

					for (int i=0; i+1<poly.pts.size(); i++) {
						auto& cmd = p.cmds.emplace_back(DrawCmd {
							.ty=DrawCmd::Line,
							.pts={poly.pts[i], poly.pts[i+1]}
						});
					}
				}
			}
		}

		nsvgDelete(img);

		if (paths.empty()) throw runtime_error("no paths");

		//reorders paths and rotates closed paths using sorted edges
		auto tsp = [](span<Path> p) {
			vector<int> out(p.size(), -1);
			vector<int> in(p.size(), -1);
			vector<optional<KDTree>> trees(p.size());
			UnionFind uf(p.size());

			struct E {
				float cost;
				int a, b, ai,bi;
				bool operator<(E const& o) const {return cost>o.cost;}
			};

			priority_queue<E> edges;

			for (int i=0; i<p.size(); i++) {
				if (!p[i].closed) continue;

				vector<Pt> endpts;
				for (auto& cmd: p[i].cmds)
					endpts.push_back(cmd.start());

				trees[i].emplace(std::move(KDTree::make(endpts)));
			}

			//for paths, this stores which end to take next, for closed paths, this stores starting cmd
			vector<int> chosen(p.size(), -1);
			vector<vector<int>> adj(p.size());

			for (int i=0; i<p.size(); i++) {
				for (int j=i+1; j<p.size(); j++) {
					if (p[i].closed && p[j].closed) {
						int nj=j, ni=i;
						if (p[nj].cmds.size() < p[ni].cmds.size()) swap(nj, ni);

						optional<pair<int,int>> closest_pair;
						float closest_dist = numeric_limits<float>::max();

						for (int ci=0; ci<p[ni].cmds.size(); ci++) {
							auto& cmd = p[ni].cmds[ci];

							KDTree* nn = trees[nj]->nearest(cmd.start()).next();
							if (nn==nullptr) continue;

							if (float d = nn->dist(cmd.start()); d<closest_dist) {
								closest_dist = d;
								closest_pair = {ci, nn->leaf_i};
							}
						}

						if (closest_pair) edges.push(E {
							closest_dist,
							ni,nj,
							closest_pair->first, closest_pair->second,
						});
					} else if (p[i].closed || p[j].closed) {
						int ni=i, nj=j;
						if (p[nj].closed) swap(ni,nj);

						for (int brev=0; brev<=1; brev++) {
							Pt x = brev ? p[nj].end() : p[nj].start();
							KDTree* nn=trees[ni]->nearest(x).next();
							if (nn) edges.push(E {
								nn->dist(x),
								ni,nj,nn->leaf_i,brev
							});
						}
					} else {
						for (int arev=0; arev<=1; arev++) {
							for (int brev=0; brev<=1; brev++) {
								Pt s = arev ? p[i].end() : p[i].start();
								Pt t = brev ? p[j].end() : p[j].start();

								edges.push(E {
									(s-t).norm(), i,j,
									arev, brev
								});
							}
						}
					}
				}
			}

			//so much BS casework...
			auto up_closed = [&](int i) {
				Pt endpt=p[i].cmds[chosen[i]].start();

				for (int j=0; j<p.size(); j++) {
					if (i==j) continue;

					if (p[j].closed) {
						if (chosen[j]==-1) {
							KDTree* nn=trees[j]->nearest(endpt).next();
							if (nn) edges.push(E {
								nn->dist(endpt),
								i,j,chosen[i],nn->leaf_i
							});
						} else {
							Pt o = p[j].cmds[chosen[j]].start();
							edges.push(E {
								(o-endpt).norm(),
								i,j,chosen[i],chosen[j]
							});
						}
					} else {
						int st = chosen[j] == -1 ? 0 : chosen[j];
						int end = chosen[j]==-1 ? 1 : chosen[j];
						for (int brev=st; brev<=end; brev++) {
							Pt t = brev ? p[j].end() : p[j].start();

							edges.push(E {
								(endpt-t).norm(), i,j,chosen[i],brev
							});
						}
					}
				}
			};

			while (edges.size()) {
				auto edge = edges.top(); edges.pop();

				if (chosen[edge.a]!=-1 && edge.ai!=chosen[edge.a]
					|| chosen[edge.b]!=-1 && edge.bi!=chosen[edge.b]
					|| adj[edge.a].size()>=2 || adj[edge.b].size()>=2) continue;

				if (uf.join(edge.a, edge.b)) {
					//should probably clean this up
					adj[edge.a].push_back(edge.b), adj[edge.b].push_back(edge.a);

					if (chosen[edge.a]==-1)
						chosen[edge.a]=p[edge.a].closed ? edge.ai : !edge.ai;
					if (chosen[edge.b]==-1)
						chosen[edge.b]=p[edge.b].closed ? edge.bi : !edge.bi;

					if (p[edge.a].closed) up_closed(edge.a);
					if (p[edge.b].closed) up_closed(edge.b);
				}
			}

			int beg=-1;
			for (int i=0; i<p.size(); i++) {
				if (adj[i].size()<=1) {
					beg=i;
					break;
				}
			}

			int i=beg,prev=-1;

			auto cont = [&]() {
				int x=i;

				if (adj[i].size() && adj[i][0]!=prev) i=adj[i][0];
				else if (adj[i].size()<=1) return false;
				else if (adj[i][1]!=prev) i=adj[i][1];

				prev=x;
				return true;
			};

			vector<int> porder(p.size());
			vector<bool> isrev(p.size(), false);
			int j=0;
			do {
				if (p[i].closed && chosen[i]!=-1) {
					rotate(p[i].cmds.begin(), p[i].cmds.begin()+chosen[i], p[i].cmds.end());
				} else if (!p[i].closed && (
						(adj[i].size()==1 && i==beg && chosen[i])
						|| (adj[i].size()>=1 && adj[i][0]==prev && !chosen[i])
						|| (adj[i].size()==2 && adj[i][1]==prev && chosen[i])
					)) {

					isrev[i]=true;
				}

				porder[j++]=i;
			} while (cont());

			auto g = [&](int i, bool x) {
				return x!=isrev[porder[i]] ? p[porder[i]].end() : p[porder[i]].start();
			};

			//TOFIX / TODO
			//doesn't seem to work
			for (int it_i=0; it_i<=2; it_i++) {
				for (int i=0; i<(int)p.size()-3; i++) {
					for (int l=i+2; l<(int)p.size()-1; l++) {
						float cur_cost = (g(i+1,false)-g(i,true)).norm()
							+ (g(l+1,false)-g(l,true)).norm();
						float new_cost = (g(l+1,false)-g(i+1,false)).norm()
							+ (g(i,true)-g(l,true)).norm();

						if (new_cost<cur_cost) {
							reverse(porder.begin()+i+1, porder.begin()+l+1);
							for (int k=i+1; k<=l; k++)
								isrev[porder[k]]=!isrev[porder[k]];
						}
					}
				}
			}

			for (int i=0; i<p.size(); i++) {
				if (isrev[i]) {
					reverse(p[i].cmds.begin(), p[i].cmds.end());
					for (DrawCmd& cmd: p[i].cmds) cmd.rev();
				}
			}

			//what in the world?
			vector<bool> visited(p.size(), false);
			for (int i=0; i<p.size(); i++) {
				if (visited[i]) continue;
				for (int j=i; porder[j]!=i; j=porder[j])
					swap(p[j],p[porder[j]]), visited[porder[j]]=true;
			}
		};

		sort(paths.begin(), paths.end(), [](Path const& a, Path const& b) {
			return a.color<b.color;
		});

		auto b=paths.begin(), e=b;
		while (b!=paths.end()) {
			while (e!=paths.end() && e->color==b->color) e++;
			tsp(span(b,e));
			b=e;
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
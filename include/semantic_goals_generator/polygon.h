/*
 * POLYGON
 *
 * Copyright (c) 2020 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_goals_generator.
 * 
 * All rights reserved.
 *
 */

/*
################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
*/

#ifndef POLYGON_H
#define POLYGON_H

#include <algorithm>
#include <limits>
#include <vector>

const double epsilon = std::numeric_limits<float>().epsilon();
const std::numeric_limits<double> DOUBLE;
const double MIN = DOUBLE.min();
const double MAX = DOUBLE.max();

struct Point{
	double x, y;
	Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
	Point(const Point& p) : x(p.x), y(p.y) {}
};

struct Edge{
	Point a, b;

	bool operator()(const Point& p) const{
		if (a.y > b.y) return Edge{ b, a }(p);
		if (p.y == a.y || p.y == b.y) return operator()({ p.x, p.y + epsilon });
		if (p.y > b.y || p.y < a.y || p.x > std::max(a.x, b.x)) return false;
		if (p.x < std::min(a.x, b.x)) return true;
		auto blue = std::abs(a.x - p.x) > MIN ? (p.y - a.y) / (p.x - a.x) : MAX;
		auto red = std::abs(a.x - b.x) > MIN ? (b.y - a.y) / (b.x - a.x) : MAX;
		return blue >= red;
	}
};

struct Polygon{
	std::string name;
	std::vector<Edge> edges;

	int size() 		{ return edges.size(); }
	void clear()	{ edges.clear(); name.clear(); }

	bool empty(){
		if( edges.size() == 0) return true;
		else return false;
	}

	bool contains(const Point& p) const{
		auto c = 0;
		for (auto e : edges) if (e(p)) c++;
		return c % 2 != 0;
	}

	/*void check(const initializer_list<geometry_msgs::Point32>& points, initializer_list<bool>& isInside) const{
		for (auto p : points)
			isInside[p] = contains(p);
	}*/
};
#endif

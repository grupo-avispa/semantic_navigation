//#include <algorithm>
//#include <cstdlib>
//#include <iomanip>
//#include <iostream>
#include <limits>
#include <geometry_msgs/Point32.h>

//using namespace std;

const double epsilon = numeric_limits<float>().epsilon();
const numeric_limits<double> DOUBLE;
const double MIN = DOUBLE.min();
const double MAX = DOUBLE.max();

//struct Point{ const double x, y;};

struct Edge{
    //const Point a, b;
    const geometry_msgs::Point32 a, b;

    //bool operator()(const Point& p) const{
    bool operator()(const geometry_msgs::Point32& p) const{
        if (a.y > b.y) return Edge{ b, a }(p);
        if (p.y == a.y || p.y == b.y) return operator()({ p.x, p.y + epsilon });
        if (p.y > b.y || p.y < a.y || p.x > max(a.x, b.x)) return false;
        if (p.x < min(a.x, b.x)) return true;
        auto blue = abs(a.x - p.x) > MIN ? (p.y - a.y) / (p.x - a.x) : MAX;
        auto red = abs(a.x - b.x) > MIN ? (b.y - a.y) / (b.x - a.x) : MAX;
        return blue >= red;
    }
};

struct Polygon{
    const string name;
    const initializer_list<Edge> edges;

    //bool contains(const Point& p) const{
    bool contains(const geometry_msgs::Point32& p) const{
        auto c = 0;
        for (auto e : edges) if (e(p)) c++;
        return c % 2 != 0;
    }

    /*void check(const initializer_list<geometry_msgs::Point32>& points, initializer_list<bool>& isInside) const{
        for (auto p : points)
            isInside[p] = contains(p);
    }*/
};


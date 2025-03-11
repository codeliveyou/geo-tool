#include <iostream>
#include <cmath>

struct Point {
    double x, y;
    Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    Point operator + (const Point &p) const {
        return Point(x + p.x, y + p.y);
    }
    Point operator-(const Point &p) const {
        return Point(x - p.x, y - p.y);
    }
    Point operator * (double factor) const {
        return Point(x * factor, y * factor);
    }
    friend Point operator * (double factor, const Point &p) {
        return p * factor;
    }
    Point operator / (double divisor) const {
        return Point(x / divisor, y / divisor);
    }
    double operator * (const Point &p) const {
        return x * p.x + y * p.y;
    }
    double operator ^ (const Point &p) const {
        return x * p.y - y * p.x;
    }
    double distanceSq(const Point &p) const {
        double dx = x - p.x;
        double dy = y - p.y;
        return dx * dx + dy * dy;
    }
    double distance(const Point &p) const {
        return std::sqrt(distanceSq(p));
    }
    double norm() const {
        return std::sqrt(x * x + y * y);
    }
    Point normalized() const {
        double n = norm();
        if (n == 0) return Point(0, 0);
        return (*this) / n;
    }
    Point rotate(double theta) const {
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);
        return Point(x * cosTheta - y * sinTheta, x * sinTheta + y * cosTheta);
    }
    void print() const {
        std::cout << "(" << x << ", " << y << ")";
    }
};

int main() {
	
    return 0;
}

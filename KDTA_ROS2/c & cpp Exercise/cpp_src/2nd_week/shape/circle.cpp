#include <cmath>
#include "circle.h"

Circle::Circle(int x, int y, int radius)
: Shape(x, y), radius_(radius) {
	
}

double Circle::area() const {
	// return 3.141592 * radius_ * radius_;
	return 3.141592 * pow(radius_, 2);
}

double Circle::getCircumference() const {
	return 3.141592 * (radius_ + radius_);
}

#include "shape.h"

Shape::Shape(int x, int y)
: x_(x), y_(y) {
}

Shape::~Shape() {

}

void Shape::move(int x, int y) {
	x_ += x;
	y_ += y;
}

/*
	double Shape::area() const {
		// Can't implement!
	}
*/

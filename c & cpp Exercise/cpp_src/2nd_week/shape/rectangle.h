#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape {
private:
	int width_;
	int height_;
	
public:
	// Rectangle() X Not use
	Rectangle(int x, int y, int w, int h);
	// Rectangle(const Rectangle& rhs);
	virtual ~Rectangle() { }
	
	// Rectangle& operator = (const Rectangle& rhs);
	
	virtual double area() const;
	double getDiagonal() const;
};

#endif

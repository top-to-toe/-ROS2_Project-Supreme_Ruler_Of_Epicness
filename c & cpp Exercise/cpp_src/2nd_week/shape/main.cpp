#include <iostream>
#include <typeinfo>
#include "shape.h"
#include "rectangle.h"
#include "circle.h"

void printShapeInfo(const Shape *ps) {
	if(typeid(*ps) == typeid(Rectangle)) {			// *** RTTI(RunTime Type Indentification) ***
		Rectangle *pr = (Rectangle *)ps;
		std::cout << "□ rectangle:\t" << pr->getDiagonal() << ",\t";
		
	} else if(typeid(*ps) == typeid(Circle)) {	// *** RTTI(RunTime Type Indentification) ***
		Circle *pc = (Circle *)ps;
		std::cout << "○ circle:\t" << pc->getCircumference() << ",\t";
		 
	} else {
		//
	}

	std::cout << "\n  area:\t\t" << ps->area() << std::endl;
}

int main() {

	// Shape s;		// abstract type!! // 추상 클래스는 객체를 만들 수 없다. 대신 포인터와 레퍼런스(참조용 객체)는 만들 수 있다.
	Shape *ps;
	
	Shape *shapes[5];
	shapes[0] = new Rectangle(10, 10, 100, 50);
	shapes[1] = new Circle(50, 50, 20);
	shapes[2] = new Rectangle(50, 50, 50, 50);
	shapes[3] = new Rectangle(200, 200, 10, 100);
	shapes[4] = new Circle(100, 100, 10);
	
	for(int i = 0; i < 5; ++i) {
		printShapeInfo(shapes[i]);
	}
	
	for(int i = 0; i < 5; ++i) {
		delete shapes[i];
	}

	return 0;
}

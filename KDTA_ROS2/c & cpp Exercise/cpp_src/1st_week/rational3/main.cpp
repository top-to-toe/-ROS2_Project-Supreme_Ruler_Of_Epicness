#include <iostream>
#include "rational.h"

int main() {

	Rational r1(3, 4);		// 3/4
	Rational r2(3);			// 3/1
	Rational r3;				// 0/1

	if(r1.getNum() == r3.getNum() && r1.getDen() == r3.getDen()) {
		std::cout << "r1 and r3 are equal" << std::endl;
	} else {
		std::cout << "r1 and r3 are not equal" << std::endl;
	}
	
	std::cout << "r1: " << r1.getNum() << "/" << r1.getDen() << std::endl;
	std::cout << "r2: " << r2.getNum() << "/" << r2.getDen() << std::endl;
	std::cout << "r3: " << r3.getNum() << "/" << r3.getDen() << std::endl << std::endl;
	
	// r3.setNum(r1.getNum());
	// r3.setDen(r1.getDen());
	r3 = r1;												// operator = (r3, r1) or r3.operator = (r1)
	
	// if(r1.getNum() == r3.getNum() && r1.getDen() == r3.getDen()) {
	if(r1 == r3) {										// operator == (r1, r3) ro r1.operator == (r3)
		std::cout << "r1 and r3 are equal" << std::endl;
	} else {
		std::cout << "r1 and r3 are not equal" << std::endl;
	}
	
	r3 = r1 + r2;
	r3 = r1 - r2;
	r3 = r1 * r2;
	r3 = r1 / r2;
	
	std::cout << "r1: " << r1.getNum() << "/" << r1.getDen() << std::endl;
	std::cout << "r2: " << r2.getNum() << "/" << r2.getDen() << std::endl;
	std::cout << "r3: " << r3.getNum() << "/" << r3.getDen() << std::endl;
	
	return 0;
}

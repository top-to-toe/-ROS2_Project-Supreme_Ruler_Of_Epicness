#include <iostream>
#include "complex.h"

int main() {
	Complex c1;							// (0, 0i)
	// Complex c2(3.0);				// (3, 0i)
	Complex c2 = 3.0;
	Complex c3(3.0, 4.0);			// (3, 4i)
	
	/*
		int i = 1;
		int i(1);
		둘다 같음.
	*/
	
	// c1.real(c3.real());
	// c1.imag(c3.imag());
	c1 = c3;								// operator = (c1, c3) or c1.operator = (c3)
	c1 = c2 = c3;						// c1.operator = (c2.operator = (c3))
	
	// if(c1.real() == c3.real() && c1.imag() == c3.imag()) {
	if(c1 == c3) {						// operator == (c1, c3) or c1.operator == (c3)
		std::cout << "c1 and c3 are equal." << std::endl;
	} else {
		std::cout << "c1 and c3 are not equal." << std::endl; 
	}
	
	c2.real(5.0);
	c2.imag(6.0);
	
	Complex c4, c5, c6, c7;
	c4 = c1 + c2;						// c1.operator + (c2) or operator + (c1, c2)
	c5 = c1 - c2;						// c1.operator - (c2) or operator - (c1, c2)
	c6 = c1 * c2;
	c7 = c1 / c2;
	
	std::cout << "c1: (" << c1.real() << ", " << c1.imag() << "i)" << std::endl;
	std::cout << "c2: (" << c2.real() << ", " << c2.imag() << "i)" << std::endl;
	std::cout << "c3: (" << c3.real() << ", " << c3.imag() << "i)" << std::endl;
	std::cout << "c4: (" << c4.real() << ", " << c4.imag() << "i)" << std::endl;
	std::cout << "c5: (" << c5.real() << ", " << c5.imag() << "i)" << std::endl;
	std::cout << "c6: (" << c6.real() << ", " << c6.imag() << "i)" << std::endl;
	std::cout << "c7: (" << c7.real() << ", " << c7.imag() << "i)" << std::endl;
	
	return 0;
}

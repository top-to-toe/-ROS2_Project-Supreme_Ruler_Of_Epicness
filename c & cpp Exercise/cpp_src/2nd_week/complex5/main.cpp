#include "complex.h"

int main() {
	Complex c1;
	Complex c2 = 3.0;
	Complex c3(3.0, 4.0);
	const Complex c4 = c3;
	
	std::cout << "c1: " << c1 << std::endl;
	std::cout << "c2: " << c2 << std::endl;
	std::cout << "c3: " << c3 << std::endl;
	std::cout << "c4: " << c3 << std::endl << std::endl;
	
	c1 += c3;
	c1 = c3;
	
	std::cout << "c1: " << c1 << std::endl;
	std::cout << "c2: " << c2 << std::endl;
	std::cout << "c3: " << c3 << std::endl;
	std::cout << "c4: " << c3 << std::endl;
	
	if(c1 == c3) {
		std::cout << "c1 and c3 are equal" << std::endl;
	} else {
		std::cout << "c1 and c3 are not equal" << std::endl;
	}
	
	c1 != c3;
	
	c1 = c2 + c3;

	++c2;
	c2++;
	// c4.real(5.0);		// read only!
	// c4.imag(6.0);
	
	std::cout << "c1: " << c1 << std::endl;
	std::cout << "c2: " << c2 << std::endl;
	std::cout << "c3: " << c3 << std::endl;
	std::cout << "c4: (" << c4.real() << ", " << c4.imag() << "i)" << std::endl;

	return 0;
}

#include <iostream>

int main() {
/* operator overloading */
	// std::cout << "Hello, World!";
	operator << (std::cout, "Hello, World!");
	//std::cout.operator << ("Hello, World!");

	// std::cout << std::endl;
	// operator << (std::cout, std::endl);
	std::cout.operator << (std::endl);
	
	// std::cout << "Hello, World!" << std::endl;
	operator << (std::cout, "Hello, World!").operator << (std::endl);
	
	return 0;
}

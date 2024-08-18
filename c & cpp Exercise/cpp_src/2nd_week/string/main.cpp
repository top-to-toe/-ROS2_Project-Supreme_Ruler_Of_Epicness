#include "string.h"

int main() {
	String s1;
	String s2 = "hello, world";
	String s3 = s2;
	
	s1 = s2;
	
	std::cout << "s1: " << s1.c_str() << std::endl;
	std::cout << "s1 len: " << s1.size() << std::endl << std::endl;
	std::cout << "s2: " << s2.c_str() << std::endl;
	std::cout << "s2 len: " << s2.size() << std::endl << std::endl;
	
	if(s1 == s2) {
		std::cout << "\t***s1 and s2 are equal***" << std::endl << std::endl;
	} else {
		std::cout << "\t***s1 and s2 are not equal***" << std::endl << std::endl;
	}
	
	std::cout << "====================================================================================================\n\n";
	std::cout << "s1: " << s1.c_str() << std::endl;
	std::cout << "s1 len: " << s1.size() << std::endl;
	
	s1 = "hello, ";
	s2 = "world!";
	
	String s4 = s1 + s2;
	
	std::cout << "s4: " << s4.c_str() << std::endl;
	std::cout << "s4 len: " << s4.size() << std::endl;
	
	std::cout << "\n====================================================================================================\n\n";
	std::cout << "s1: " << s1 << std::endl;
	std::cout << "s2: " << s2 << std::endl;
	std::cout << "s3: " << s3 << std::endl;
	std::cout << "s4: " << s4 << std::endl;
	
	return 0;
}

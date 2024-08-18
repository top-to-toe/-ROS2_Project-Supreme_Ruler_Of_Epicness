#include "stack.h"

int main() {
	Stack<int> s1;
	s1.push(100);
	s1.push(200);
	s1.push(300);
	
	while(!s1.empty()) {
		std::cout << s1.pop() << std::endl;
	}
	
	Stack<double> s2;
	s2.push(1.1);
	s2.push(2.2);
	s2.push(3.3);
	
	while(!s2.empty()) {
		std::cout << s2.pop() << std::endl;
	}
	
	return 0;
}

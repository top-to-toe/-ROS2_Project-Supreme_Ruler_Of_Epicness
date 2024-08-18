#include <stdio.h>

int main(void) {
	int a, b;
	a = 100;
	b = 200;
	printf("a: %d\tb: %d\n", a, b);
	
	// swap
	a = b;
	b = a;
	
	printf("a: %d\tb: %d\n", a, b);
	return 0;
}

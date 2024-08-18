#include <stdio.h>

int main(void) {

	int a;
	int *p;
	
	a = 100;
	p = &a;		//	p = a's address value
	*p = 200;	// == a = 200;
	
	printf("p(a's address): %p\n", p);
	printf("a: %d\n", a);
	return 0;
}

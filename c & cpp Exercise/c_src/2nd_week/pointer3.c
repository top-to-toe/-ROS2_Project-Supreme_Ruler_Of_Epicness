#include <stdio.h>

int main(void) {

	/*
	int a = 0x12345678;
	
	// int *p = &a;
	char *p  = (char*)&a;
	
	printf("*p: 0x%x\n", *p);		// 결과는 0x78이 출력됨.(CPU 때문에...little endian)
	*/


	int a;
	int *pa;
	int **ppa;
	
	a = 100;
	pa = &a;
	ppa = &pa;
	**ppa = 200;
	
	// 역참조(Dereference)
	// ppa == &pa (int**)
	// *ppa == pa == &a (int*)
	// **ppa == *pa == a (int)
	
	printf("%d\n", a);
	return 0;
}

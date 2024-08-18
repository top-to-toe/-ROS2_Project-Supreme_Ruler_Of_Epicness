#include <stdio.h>

int main(void) {
	int a = 0x12345678;
	int b = 0x0000FFFF;
	
	int c = a & b;
	
	printf("c: 0x%08x\n", c);
	return 0;
}

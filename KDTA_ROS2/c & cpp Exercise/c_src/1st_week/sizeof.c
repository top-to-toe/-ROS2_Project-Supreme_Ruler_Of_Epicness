#include <stdio.h>

int main(void) {
	printf("sizeof(char): %ld\n", sizeof(char));
	printf("sizeof(short): %ld\n", sizeof(short));
	printf("sizeof(int): %ld\n", sizeof(int));
	printf("sizeof(long): %ld\n", sizeof(long));
	printf("sizeof(long long): %ld\n", sizeof(long long));
	
	printf("sizeof(float): %ld\n", sizeof(float));
	printf("sizeof(double): %ld\n", sizeof(double));
	printf("sizeof(long double): %ld\n", sizeof(long double));
	
	printf("sizeof(void): %ld\n", sizeof(void));					// maybe is not used.
	
	int i;
	printf("sizeof(i): %ld\n", sizeof(i));
	
	printf("sizeof(3.141592): %ld\n", sizeof(3.141592));
	printf("sizeof('A'): %ld\n", sizeof('A'));
	
	return 0;
}

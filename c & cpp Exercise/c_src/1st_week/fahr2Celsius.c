#include <stdio.h>

int main(void) {
	int fahr;
	int celsius;
	
	fahr = 100;
	
	celsius = 5 * (fahr - 32) / 9;
	
	printf("fahr: %d ---> celsius: %d\n", fahr, celsius);
	return 0;
}

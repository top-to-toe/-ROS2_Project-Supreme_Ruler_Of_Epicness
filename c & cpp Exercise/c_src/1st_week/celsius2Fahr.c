#include <stdio.h>

int main(void) {
	int celsius;
	double fahr;
	
	// celsius = 40;
	scanf("%d", &celsius);	
	fahr = 9.0 / 5.0 * celsius + 32;
	
	printf("celsius: %d ---> fahr: %.1f\n", celsius, fahr);	
	return 0;
}

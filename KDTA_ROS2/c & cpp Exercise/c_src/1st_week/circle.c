#include <stdio.h>
#define PI 3.141592

int main(void) {
	// int radius;
	double radius;
	
	printf("input radius: ");
	// scanf("%d", &radius);
	scanf("%lf", &radius);
	
	double area = radius * radius * PI;
	printf("area: %.2f\n", area);
	return 0;
}

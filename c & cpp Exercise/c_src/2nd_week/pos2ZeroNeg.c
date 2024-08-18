#include <stdio.h>

int main(void) {
	
	int num;
	printf("input num: ");
	scanf("%d", &num);
	
	// positive or zero or negative
	/*
	if(num > 0) {
		printf("%d is positive number\n", num);
	} else if(num == 0) {
		printf("%d is zero number\n", num);
	} else {
		printf("%d is negative number\n", num);
	}
	*/
	
	printf("%d is a %s number\n", num, (num > 0) ? "positive" : (num == 0) ? "zero" : "negative");

	return 0;
}

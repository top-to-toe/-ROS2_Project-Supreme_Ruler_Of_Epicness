#include <stdio.h>

int main(void) {
	
	int num;
	printf("input num: ");
	scanf("%d", &num);
	
	// int isOdd = (num % 2 == 1);
	// int isOdd = (num % 2);
	// int isOdd = !(num % 2 == 0);
	int isOdd = (num % 2 != 0);
	printf("%d is a odd number : %d\n", num, isOdd);
	return 0;
}

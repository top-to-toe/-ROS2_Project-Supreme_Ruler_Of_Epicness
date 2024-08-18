#include <stdio.h>

int main(void) {
	
	int num;
	printf("input num: ");
	scanf("%d", &num);
	
	int absoluteValue;
	absoluteValue = (num >= 0) ? num : -num;
	/*
	if(num >= 0) {
		absoluteValue = num;
	} else {
		absoluteValue = -num;
	}
	*/
	
	printf("abs value: %d\n", absoluteValue);
	return 0;
}

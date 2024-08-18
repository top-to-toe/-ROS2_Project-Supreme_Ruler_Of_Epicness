#include <stdio.h>

int main(void) {

	int num;
	printf("input num : ");
	scanf("%d", &num);
	
	int re = (num >= 1 && num <= 9);
	
	printf("%d is [1, 9] : %d\n", num, re);

	return 0;
}

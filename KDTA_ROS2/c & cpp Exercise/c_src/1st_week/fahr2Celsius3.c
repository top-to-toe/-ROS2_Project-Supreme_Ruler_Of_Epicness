#include <stdio.h>

int main(void) {
	int fahr;
	fahr = 100;
	
	int celsius1000;
	celsius1000 = 1000 * 5 * (fahr - 32) / 9;					// 37,777
	
	int left = celsius1000 / 1000;								// 37
	// int right = (celsius1000 - celsius1000 / 1000 * 1000 + 5) / 10;	// 나머지 연산 사용전
	int right = (celsius1000 % 1000 + 5) / 10;		// 나머지 연산 사용후
	
	printf("fahr: %d ---> celsius: %d.%d\n", fahr, left, right);
	return 0;
}

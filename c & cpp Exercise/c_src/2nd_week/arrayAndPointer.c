#include <stdio.h>

int main(void) {
	// 포인터와 배열의 관계(repeat)
	int nums[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	int *p;
	
	p = nums;						// p = &nums[0];
	
	printf("*p: %d\n", *p);		// *p: 1 print.
	
	++p;								// p = p + 1
	printf("*p: %d\n", *p);		// *p: 2 print.
	
	p = nums;
	for(int i = 0; i < 10; ++i) {
		printf("*(p + %d): %d\n", i+1, *(p + i));		// index에 대응하는 배열 내용 위치의 값. 
	}
	return 0;
}

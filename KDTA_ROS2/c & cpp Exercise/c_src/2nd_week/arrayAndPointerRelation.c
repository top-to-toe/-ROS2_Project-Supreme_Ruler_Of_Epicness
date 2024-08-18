#include <stdio.h>

int main(void) {

	int nums[] = {1, 2, 3, 4, 5};
	int *p;
	p = nums;
	
	// 1. p == nums
	/*
		p + 0 == &nums[0]
		p + 1 == &nums[1]
		p + 2 == &nums[2]
		p + 3 == &nums[3]
		...
		p + i == &nums[i]
	*/

	
	// 2-1. nums[i]	==	*(nums + i)
	//	2-2. *(p + i)	==	p[i]
	/*
		*(p+0) == nums[0]
		*(p+1) == nums[1]
		*(p+2) == nums[2]
		*(p+3) == nums[3]
		...
		*(p+i) == nums[i]
	*/

	// 포인터는 배열식으로, 배열은 포인터식으로 사용할 수 있다.
	return 0;
}

#include <stdio.h>

int main(void) {
	// arrayAndPointerRealation.c (Repeat)
	int nums[] = {1, 2, 3, 4, 5};
	int *p;
	
	p = nums;
	
	for(int i = 0; i < 5; ++i) {
		printf("%d ", nums[i]);			// nums[i]
	}
	printf("\n");
	
	for(int i = 0; i < 5; ++i) {
		printf("%d ", *(nums + i));	// *(nums + i)		포인터로 배열식 사용.
	}
	printf("\n");
	
	for(int i = 0; i < 5; ++i) {
		printf("%d ", *(p + i));		// *(p + i)
	}
	printf("\n");
	
	for(int i = 0; i < 5; ++i) {
		printf("%d ", p[i]);				// p[i]				배열로 포인터식 사용.
	}
	printf("\n");
	
	return 0;
}

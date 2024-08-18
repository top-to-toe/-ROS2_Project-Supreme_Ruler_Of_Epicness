#include <stdio.h>

int main(void) {

	int nums1[] = {1, 2, 3, 4, 5};
	int nums2[] = {1, 2, 3, 4, 5};

	/*
	if(nums1 == nums2) {			// &nums1[0] == &nums2[0]
		printf("nums1 and nums2 are equal\n");
	} else {
		printf("nums1 and nums2 are not equal\n");
	}
	*/
	
	int i;
	for(i = 0; i < 5; ++i) {
		if(nums1[i] != nums2[i]) {
			break;
		} else
			printf("nums1[%d]: %d, \tnums2[%d]: %d\t", i+1, nums1[i], i+1, nums2[i]);
			if(nums1[i] == nums2[i]) {
				printf("(EQUAL!!!)\n");
			}
	}
	
	if(i == 5) {
		// equal
		printf("nums1 and nums2 are equal\n");
	} else {
		// not equal
		printf("nums1 and nums2 are not equal\n");
	}
	
	return 0;
}

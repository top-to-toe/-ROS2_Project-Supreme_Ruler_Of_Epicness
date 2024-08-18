// 배열과 포인터의 관계

#include <stdio.h>

int main(void) {
	
	int nums[5] = {1, 2, 3, 4, 5};
	int *p;
	p = nums;		// == p = &nums[0]; 배열이름이 식에서 사용되면 첫번째 element의 시작 주소를 의미한다.
	
	
	// nums의 주소값을 nums[0]: 1000, nums[1]: 1004, nums[2]: 1008 ....라고 가정하면..
	p = p + 1;		// 1002(X) 1004(O) ===> 포인터의 +1은 다음 element(주소.공간)를 가리킴.(+2는 다음다음 element) --> 배열의 index
	// 더해지는 index 상수값 만큼 가리키는 element에 대해 다르게 접근.(만약 1001로 가려면 char로, 1008로 가려면 double로 +1을 하면 가능
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////
	
	int nums1[] = {1, 2, 3, 4, 5};
	int nums2[5];
	
	// nums2 = nums1;		==> &num2[0] = &nums1[0]; (ERROR!!!) C언어는 배열 간 치환 연산 불가능!(다른 언어는 가능)
	// 해결책
	/*
	for(int i = 0; i < 5; ++i) {
		nums2[i] = nums1[i];
	}
	*/
	// 이런식으로 반복문을 활용하여 배열의 각각 위치를 매칭화하여 접근토록 해야한다.
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////
	
	int nums1[5] = {1, 2, 3, 4, 5};
	int nums2[5] = {1, 2, 3, 4, 5};
	
	// nums1 == nums2; ==> &nums1[0] == &nums2[0] ???
	// 비교가 가능하냐고? Nope! FALSE! 이런 식으론 배열 간 내용의 비교구문하는 것이 아닌 배열의 메모리상 위치간 비교를 하는 것이기에 의도한 바와 다른 식이 된다.
	// 해결책
	/*
	for(int i = 0 ; i < 5; ++i) {
		if(nums1[i] == nums2[i]) {
			...
		}
	}
	*/ 결국 반복문 및 조건문을 통해서만 접근 가능
	return 0;
}

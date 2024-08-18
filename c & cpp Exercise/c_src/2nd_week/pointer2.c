#include <stdio.h>

int main(void) {

	int num;
	int nums[5];
	
	int *p;
	p = NULL;
	p = &num;
	p = &num[5];
	
	*p = 100;
	
	printf("*p: %\n", *p);
	
	// *** 포인터는 반드시 변수나 배열의 element를 가리켜야 한다. ***

	/*
	int a = 0x12345678;
	char *p;
	
	p = (char*)&a;		//	p = a's address value
	
	printf("%x\n", *p);
	*/
	return 0;
}

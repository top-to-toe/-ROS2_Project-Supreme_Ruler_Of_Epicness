#include <stdio.h>

/*
	<--- void pointer --->
	포괄형(generic) pointer (순수하게 주소값 저장용)
*/

/*

	int a = 100;
	void *p;
	p = &a;
	
	※	일반적인 형태로 역참조를 할 수 없으므로 다음과 같은 형태가 고정적이다.			
	*(int*)p = 200;
	
*/

/*

	double d = 3.14;
	void *p;
	p = &d;
	
	※	일반적인 형태로 역참조를 할 수 없으므로 다음과 같은 형태가 고정적이다.
	*(double*)p = 2.718;
	
*/

int main(void) {

	return 0;
}

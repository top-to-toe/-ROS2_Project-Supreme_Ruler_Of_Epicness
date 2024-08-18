// #include <stdlib.h>
#include <cstdlib>
#include <assert.h>
#include "stack.h"

using namespace std;

// void Stack::initStack(/* Stack *ps, */int size) {
Stack::Stack(int size) {
	// ps->pArr = (int *)malloc(sizeof(int) * size);
	// ps->size = size;
	// ps->tos = 0;
	
	// this->pArr = (int *)malloc(sizeof(int) * size);
	this->pArr = new int[size];
	assert(this->pArr ); 											// NULL인지 아닌지 체크
	this->size = size;
	this->tos = 0;
}

// void Stack::cleanupStack(/* Stack *ps */) {
Stack::~Stack() {
	// free(ps->pArr);
	
	// free(this->pArr);
	delete [] this->pArr;
}

void Stack::push(/* Stack *ps, */int data) {
	// assert(ps->tos != ps->size);
	// ps->pArr[ps->tos] = data;
	// ++ps->tos;
	
	assert(this->tos != this->size);
	this->pArr[this->tos] = data;
	++this->tos;
}

int Stack::pop(/* Stack *ps */) {
	// if(ps->tos == ps->size) {
	//	fprintf(stderr, "stack is full\n");
	// exit(1);
	// }
	
	// assert(ps->tos != 0);
	// --ps->tos;
	// return ps->pArr[ps->tos];
	assert(this->tos != 0);
	--this->tos;
	return this->pArr[this->tos];
}

/////////////////////* C++ */////////////////////
// .c --> .cpp

// OOP? push(&s1, 100); ---> s1.push(100)
// > 전역 함수 --> 멤버함수
// * 멤버함수는 반드시 객체(구조체 변수)를 통해서 호출한다.
//   s1.push(100);	s2.push(200);

// * 멤버함수는 자신을 호출한 객체를 가리키는 this라는 포인터를 가진다.
// > ps-> ---> this->
// > ※ 멤버함수를 위한 공간은 있어야 되지않나?? X ==> 멤버 자료(변수 및 배열)만을 위한 공간만... this포인터를 활용

// malloc() / free() --> new / delete 연산자
// 생성자(constructor): 객체가 생성될 때 자동적으로 호출되는 멤버함수
// 소멸자(destructor): 객체가 소멸된 때 자동적으로 호출되는 멤버함수
// > void Stack::initStack(int size) ---> Stack::Stack(int size)
// > void Stack::cleanupStack() ---> Stack::~Stack()


// 접근지정자(private, public)
// > 내부 구현(implementation)은 숨기고, 인터페이스(interface)는 보인다.
// > 허용된 연산을 통해서만 내부 구현에 접근한다.

// struct --> class
// > class(C++) = C's struct + member func. + private/pulbic

// <stdio.h> ---> namespace가 적용된 <cstdio>

// printf() ---> cout << ...;

// int main(void) ---> int main()
// typedef struct {} Struct; ---> struct Stack {};

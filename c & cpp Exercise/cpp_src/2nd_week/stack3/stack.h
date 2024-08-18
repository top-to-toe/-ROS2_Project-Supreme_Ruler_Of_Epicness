#ifndef STACK_H
#define STACK_H
// #define STACKSIZE 100

#include "array.h"

// extern const int STACKSIZE;		// 특정 클래스와 관련된 전역 변수

class Stack {

private:
	static const int STACKSIZE;
	
	// int *pArr_;
	// int size_;
	Array arr_;				// 'has-a'관계 
	int tos_;
	
	Stack(const Stack& rhs);						// Can't use.
	Stack& operator=(const Stack& rhs);			// Can't use.
	
public:
	explicit Stack(int size = STACKSIZE);
	~Stack();
   
   // Stack *operator & () { return this; }
   // const Stack *operator & () const { return this; }
   
	void push(int data);
	int pop();
	
	bool full() const;
   bool empty() const;
   
};

#endif

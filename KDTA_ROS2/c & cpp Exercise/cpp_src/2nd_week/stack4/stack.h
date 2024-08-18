#ifndef STACK_H
#define STACK_H

#include "array.h"

template <typename T>
class Stack {

private:
	static const int STACKSIZE;
	
	Array<T> arr_;				// 'has-a'관계 
	int tos_;
	
	Stack(const Stack<T>& rhs);						// Can't use.
	Stack<T>& operator=(const Stack<T>& rhs);			// Can't use.
	
public:
	explicit Stack(int size = STACKSIZE);
	~Stack();
   
   // Stack<T> *operator & () { return this; }
   // const Stack<T> *operator & () const { return this; }
   
	void push(const T& data);
	const T pop();
	
	bool full() const;
   bool empty() const;
   
};

#include <cassert>

template <typename T>
const int Stack<T>::STACKSIZE = 100;

template <typename T>
Stack<T>::Stack(int size)
: arr_(size), tos_(0) {
	
}

template <typename T>
Stack<T>::~Stack() {

}

template <typename T>
void Stack<T>::push(const T& data) {
	assert(!this->full());
	
	arr_[tos_] = data;
	++tos_;
}

template <typename T>
const T Stack<T>::pop() {
	assert(!this->empty());
	
	--tos_;
	return arr_[tos_];
}

template <typename T>
bool Stack<T>::full() const {
	return tos_ == arr_.size();
}

template <typename T>
bool Stack<T>::empty() const {
	return tos_ == 0;
}

#endif

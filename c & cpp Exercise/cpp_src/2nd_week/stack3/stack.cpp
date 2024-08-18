// #include <cstdlib>
#include <cassert>
#include "stack.h"

// extern const int Stack::STACKSIZE = 100;
const int Stack::STACKSIZE = 100;

/*
Stack::Stack(int size)
: pArr_(new int[size]), size_(size), tos_(0) {
	// pArr_ = new int[size];
	assert(pArr_);
	// size_ = size;
	// tos_ = 0;
}
*/

/*
Stack::~Stack() {
	delete [] pArr_;
}
*/

Stack::Stack(int size)
: arr_(size), tos_(0) {
	
}

Stack::~Stack() {
	// arr_.~Array() is automatically called
}

void Stack::push(int data) {
	// assert(tos_ != size_);
	assert(!this->full());
	
	// pArr_[tos_] = data;
	arr_[tos_] = data;
	++tos_;
}

int Stack::pop() {
	// assert(tos_!= 0);
	assert(!this->empty());
	
	--tos_;
	// return pArr_[tos_];
	return arr_[tos_];
}

bool Stack::full() const {
	// return tos_ == size_;
	return tos_ == arr_.size();
}

bool Stack::empty() const {
	return tos_ == 0;
}

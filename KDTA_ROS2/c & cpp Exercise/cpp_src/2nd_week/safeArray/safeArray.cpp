#include <cassert>
#include "safeArray.h"

SafeArray::SafeArray(int size)
: Array(size) {

}

SafeArray::SafeArray(const int *pArr, int size)
: Array(pArr, size) {

}

SafeArray::SafeArray(const SafeArray& rhs)
: Array( (Array)rhs ) {			// slicing

}

SafeArray::~SafeArray() {
	// Array::~Array() is automatically called.
}


SafeArray& SafeArray::operator = (const SafeArray& rhs) {
	this->Array::operator = ( (Array)rhs );

	return *this;
}

bool SafeArray::operator == (const SafeArray& rhs) const {
	return this->Array::operator == ( (Array)rhs );
}

int& SafeArray::operator [] (int index) {
	// boundary check!
	assert(index >= 0 && index < this->size());							// this->Array::size()
	
	return this->Array::operator [] (index);
}

const int& SafeArray::operator [] (int index) const {
	// boundary check!
	assert(index >= 0 && index < this->size());							// this->Array::size()

	return this->Array::operator [] (index);
}

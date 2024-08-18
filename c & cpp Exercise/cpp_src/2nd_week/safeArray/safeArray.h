#ifndef SAFEARRAY_H
#define SAFEARRAY_H
#include "array.h"

class SafeArray : public Array {				// 'is-a'
private:

public:
	explicit SafeArray(int size = Array::ARRAYSIZE);
	// explicit SafeArray(int size = Array::getArrayDefaultSize());
	SafeArray(const int *pArr, int size);
	SafeArray(const SafeArray& rhs);
	virtual ~SafeArray();

	SafeArray& operator = (const SafeArray& rhs);
	
	bool operator == (const SafeArray& rhs) const;

	virtual int& operator [] (int index);
	virtual const int& operator [] (int index) const;
	
	// SafeArray *operator& () { return this; }
	// const SafeArray *operator& () const { return this; }
	
	
	
};

#endif

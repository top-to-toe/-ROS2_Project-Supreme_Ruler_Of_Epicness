#ifndef ARRAY_H
#define ARRAY_H
// #define ARRAYSIZE 100

// extern const int ARRAYSIZE; 						// 외부에서 사용한다는 의미(보통 헤더에서 정의)

#include <iostream>

class Array {

friend std::ostream& operator << (std::ostream& out, const Array& rhs);

private:
	static const int ARRAYSIZE;
	int *pArr_;
	int size_;

public:
	explicit Array(int size = ARRAYSIZE);			// int 타입 객체로 Array타입 객체로 묵시적으로 변환되는것을 막음(explicit 사용)
	Array(const int *pArr, int size);
	Array(const Array& rhs);
	~Array();
	
	// Array *operator & () { return this; }
	// const Array *operator & () const { return this; }
	
	Array& operator = (const Array& rhs);
	
	bool operator == (const Array& rhs) const;
	// bool operator != (const Array& rhs) const;
	
	int& operator [] (int index);
	const int& operator [] (int index) const;
	
	int size() const;
};

#endif

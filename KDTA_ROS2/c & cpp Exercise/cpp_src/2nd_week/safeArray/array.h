#ifndef ARRAY_H
#define ARRAY_H
// #define ARRAYSIZE 100

// extern const int ARRAYSIZE; 						// 외부에서 사용한다는 의미(보통 헤더에서 정의)

#include <iostream>

class Array {

friend std::ostream& operator << (std::ostream& out, const Array& rhs);

private:
	// static const int ARRAYSIZE;
	int *pArr_;
	// int size_;

protected:
	static const int ARRAYSIZE;
	
	int size_;

public:
	static int getArrayDefaultSize();		// array클래스와 관련된 전역함수(static)
	
	explicit Array(int size = ARRAYSIZE);			// int 타입 객체로 Array타입 객체로 묵시적으로 변환되는것을 막음(explicit 사용)
	Array(const int *pArr, int size);
	Array(const Array& rhs);
	virtual ~Array();
	
	// Array *operator & () { return this; }
	// const Array *operator & () const { return this; }
	
	Array& operator = (const Array& rhs);
	
	bool operator == (const Array& rhs) const;
	// bool operator != (const Array& rhs) const;
	
	virtual int& operator [] (int index);				// virtual - 동적 바인딩을 위한 키워드(반드시 소멸자도 virtual 사용해야됨!)
	virtual const int& operator [] (int index) const;
	
	int size() const;
};

#endif

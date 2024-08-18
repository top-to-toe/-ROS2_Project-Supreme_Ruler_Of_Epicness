#ifndef EMPTY_H
#define EMPTY_H

class Empty {
private:

public:
	/***** 멤버함수가 정의되지 않는 경우, 컴파일러가 자동적으로 제공하는 멤버함수는 6개 *****/
	// Empty() { }					// if other constructor. X(다른 생성자가 없으면 본체가 비어있는 기본 생성자 만들어줌.) 제약조건 X
	// Empty(const Empty& rhs) { /*memberwise copy*/ }
	// ~Empty() { }				// if other destructor. X(소멸자 또한 기본생성자와 동일하게 만들어줌.)
	
	// Empty& operator=(const Empty& rhs) { /* memberwise copy; return *this; */ }	// 치환 연산
	
	// Empty* operator&() {	return this; }						// 일반 객체에 대해서..
	// const Empty* operator&() const { return this; }		// 상수 객체에 대해서..
};

#endif

#ifndef RATIONAL_H
#define RATIONAL_H

#include <iostream>

class Rational {

friend std::ostream& operator << (std::ostream &out, const Rational& rhs);		// 전역 함수! 멤버 함수 X
friend std::istream& operator >> (std::istream &in, Rational& rhs);

private:
	int num;						// numerator
	int den;						// denominator

	void abbreviate();		// 약분하는 함수(내부 구현과 관련된 함수이므로 private 영역 배치)
									// tool function OR helper function.
public:
	
	// Rational();
	// Rational(int num);
	Rational(int num = 0, int den = 1);		// default argument.
	Rational(const Rational &rhs);			// copy constructor.
	~Rational();
	
	Rational& operator = (const Rational& rhs);
	bool operator == (const Rational& rhs);
	
	const Rational operator + (const Rational& rhs);
	const Rational operator - (const Rational& rhs);
	const Rational operator * (const Rational& rhs);
	const Rational operator / (const Rational& rhs);

	//
	int getNum();
	int getDen();
	
	void setNum(int num);
	void setDen(int den);
};

#endif

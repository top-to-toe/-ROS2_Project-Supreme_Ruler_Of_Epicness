#ifndef COMPLEX_H
#define COMPLEX_H

#include <iostream>

class Complex;
std::ostream& operator << (std::ostream& out, const Complex& rhs);

class Complex {

private:
	double re_;
	double im_;

public:
	// Complex();
	// Complex(double re);
	Complex(double re = 0.0, double im = 0.0);
	Complex(const Complex& rhs);
	~Complex();
	
	Complex& operator = (const Complex& rhs);
	Complex& operator += (const Complex& rhs);
	// Complex& operator -= (const Complex& rhs);
	// Complex& operator %= (const Complex& rhs);
	// %= X
	
	// &=, |=, <<=, >>= X
	
	const Complex operator + (const Complex& rhs) const;		// 자기 자신에 대해서 바뀌지 않는 경우(read only) 함수 뒷 부분 const 키워드 추가
	// const Complex operator - (const Complex& rhs);
	// const Complex operator * (const Complex& rhs);
	// const Complex operator / (const Complex& rhs);
	
	// overloading 연습용
	const Complex& operator ++ ();			// prefix
	const Complex operator ++ (int );		// postfix
	// ++, -- X
	
	bool operator == (const Complex& rhs) const;
	bool operator != (const Complex& rhs) const;
	// bool operator > (const Complex& rhs);
	// bool operator < (const Complex& rhs);
	// bool operator >= (const Complex& rhs);
	// bool operator <= (const Complex& rhs);
	
	double real() const;
	double imag() const;
	
	void real(double re);
	void imag(double im);
	
};

#endif

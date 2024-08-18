#include "complex.h"

Complex::Complex() {
	this->re = 0.0;
	this->im = 0.0;
}

Complex::Complex(double re) {
	this->re = re;
	this->im = 0.0;
}

Complex::Complex(double re, double im) {
	this->re = re;
	this->im = im;
}

Complex::~Complex() {
	
}

/*
void Complex::operator = (const Complex& rc) {
	this->re = rc.re;
	this->im = rc.im;
}
*/

Complex& Complex::operator = (const Complex& rc) {
	this->re = rc.re;
	this->im = rc.im;
	
	return *this;
}

bool Complex::operator == (const Complex& rc) {
	/*
	if(this->re == rc.re && this->im == rc.im) {
		return true;
	} else {
		return false;
	}
	*/
	return (this->re == rc.re && this->im == rc.im);
}

// 사칙연산 operator 구현시 값 전달방식 채택
// (지역변수를 반환하기때문에 포인터 또는 레퍼런스를 사용시 가리키는 변수의 공간이 사라지는 경우 위험!)

const Complex Complex::operator + (const Complex& rc) {
	Complex result(this->re + rc.re, this->im + rc.im);
	
	return result;
}

const Complex Complex::operator - (const Complex& rc) {
	Complex result(this->re - rc.re, this->im - rc.im);
	
	return result;
}

const Complex Complex::operator * (const Complex& rc) {
	Complex result((this->re * rc.re)-(this->im * rc.im), (this->re * rc.im) + (this->im * rc.re));
	
	return result;
}

const Complex Complex::operator / (const Complex& rc) {
	Complex result(((this->re * rc.re) + (this->im * rc.im)) / ((rc.re * rc.re) + (rc.im * rc.im)), ((this->im * rc.re) - (this->re * rc.im)) / ((rc.re * rc.re) + (rc.im * rc.im)));
	
	return result;
}

double Complex::real() {
	return this->re;
}

double Complex::imag() {
	return this->im;
}

void Complex::real(double re) {
	this->re = re;
}

void Complex::imag(double im) {
	this->im = im;
}

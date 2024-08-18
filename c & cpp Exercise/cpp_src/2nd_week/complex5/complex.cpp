#include "complex.h"

std::ostream& operator << (std::ostream& out, const Complex& rhs) {
	return out << "(" << rhs.real() << ", " << rhs.imag() << "i)";
}

/*
Complex::Complex()
: re_(0.0), im_(0.0) {		// 생성자 초기화 리스트(constructor initialization list.)
	// re_ = 0.0;
	// im_ = 0.0;
}

Complex::Complex(double re)
: re_(re), im_(0.0) {
	// re_ = re;
	// im_ = 0.0;
}
*/

Complex::Complex(double re, double im)
: re_(re), im_(im) {
	// re_ = re;
	// im_ = im;
}

Complex::Complex(const Complex& rhs)
: re_(rhs.re_), im_(rhs.im_) {
	// re_ = rhs.re_;
	// im_ = rhs.im_;
}

Complex::~Complex() {

}


Complex& Complex::operator = (const Complex& rhs) {
	re_ = rhs.re_;
	im_ = rhs.im_;
	
	return *this;
}

Complex& Complex::operator += (const Complex& rhs) {
	re_ += rhs.re_;
	im_ += rhs.im_;
	
	return *this;
}

const Complex Complex::operator + (const Complex& rhs) const {
	Complex result(re_ + rhs.re_, im_ + rhs.im_);
	
	return result;
}

const Complex& Complex::operator ++ () {
	re_ = re_ + 1.0;
	im_ = im_ + 1.0;
	
	return *this;
}

const Complex Complex::operator ++ (int ) {
	Complex tmp = *this;
	re_ = re_ + 1.0;
	im_ = im_ + 1.0;
	
	return tmp;
}

bool Complex::operator == (const Complex& rhs) const {
	return re_ == rhs.re_ && im_ == rhs.im_;
}

bool Complex::operator != (const Complex& rhs) const {
	// return !this->operator == (rhs);				// 위 구현된 함수 활용 version.
	return re_ != rhs.re_ || im_ != rhs.im_;
}

double Complex::real() const {
	return re_;
}

double Complex::imag() const {
	return im_;
}

void Complex::real(double re) {
	re_ = re;
}

void Complex::imag(double im) {
	im_ = im;
}

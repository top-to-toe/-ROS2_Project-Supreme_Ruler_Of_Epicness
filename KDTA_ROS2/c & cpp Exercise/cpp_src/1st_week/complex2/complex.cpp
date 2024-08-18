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

void Complex::assign(Complex c) {
	this->re = c.re;
	this->im = c.im;
}

void Complex::assign2(const Complex *pc) {
	this->re = pc->re;
	this->im = pc->im;
}

void Complex::assign3(const Complex &rc) {
	this->re = rc.re;
	this->im = rc.im;
}

int Complex::equals(const Complex &rc) {
	if(this->re == rc.re &&	this->im == rc.im) {
		// return 1;
		return true;
	} else {
		// return 0;
		return false;
	}
}

// get part function.
double Complex::real() {
	return this->re;
}

double Complex::imag() {
	return this->im;
}

// set part function.
void Complex::real(double re) {
	this->re = re;
}

void Complex::imag(double im) {
	this->im = im;
}

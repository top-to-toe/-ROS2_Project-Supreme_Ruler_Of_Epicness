#include <cassert>
#include "rational.h"

Rational::Rational() {
	this->num = 0;
	this->den = 1;
}

Rational::Rational(int num) {
	this->num = num;
	this->den = 1;
}

Rational::Rational(int num, int den) {
	assert(den != 0);			// 유리수 조건
	
	this->num = num;
	this->den = den;
}

Rational::~Rational() {

}

Rational& Rational::operator = (const Rational& rr) {
	this->num = rr.num;
	this->den = rr.den;
	
	return *this;
}

bool Rational::operator == (const Rational& rr) {
	return (this->num == rr.num && this->den == rr.den);
}

bool Rational::operator != (const Rational& rr) {
	return (this->num != rr.num || this->den != rr.den);
}

int Rational::getNum() {
	return this->num;
}

int Rational::getDen() {
	return this->den;
}

void Rational::setNum(int num) {
	this->num = num;
}

void Rational::setDen(int den) {
	this->den = den;
}

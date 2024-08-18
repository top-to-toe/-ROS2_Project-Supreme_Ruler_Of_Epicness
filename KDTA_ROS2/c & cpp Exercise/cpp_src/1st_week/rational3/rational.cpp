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

Rational& Rational::operator = (const Rational& rhs) {
	this->num = rhs.num;
	this->den = rhs.den;
	
	return *this;
}

bool Rational::operator == (const Rational& rhs) {
	return this->num == rhs.num && this->den == rhs.den;
}

const Rational Rational::operator + (const Rational& rhs) {
	Rational result(this->num * rhs.den + rhs.num *rhs.den, this->den * rhs.den);
	
	return result;
}

const Rational Rational::operator - (const Rational& rhs) {
	Rational result(this->num * rhs.den - rhs.num *rhs.den, this->den * rhs.den);
	
	return result;
}

const Rational Rational::operator * (const Rational& rhs) {
	Rational result(this->num * rhs.num, this->den * rhs.den);
	
	return result;
}

const Rational Rational::operator / (const Rational& rhs) {
	Rational result(this->num * rhs.den, this->den * rhs.num);
	
	return result;
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

#ifndef COMPLEX_H
#define COMPLEX_H

class Complex {
private:
	// implementation
	double re;				// 실수부
	double im;				// 허수부
	
public:
	// interface
	Complex();
	Complex(double re);
	Complex(double re, double im);
/* function overloading.(함수(이름)중복 - 제약조건 => 단 형태가 달라야함.) -->(사용이유) one-interface multi-method */
	~Complex();	
	// 
	double real();
	double imag();
	
	void real(double re);
	void imag(double im);
};

#endif

#ifndef STRING_H
#define STRING_H

#include <iostream>

class String {
friend std::ostream& operator << (std::ostream& out, const String& rhs);
private:
	char *str_;
	int len_;
	
public:
	String();
	String(const char *str);
	String(const String& rhs);
	~String();
	
	String& operator = (const String& rhs);
	// String& operator = (const String& rhs);
	// -=, *=, /=, %=
	
	bool operator == (const String& rhs) const;
	// bool operator != (const String& rhs) const;
	// >, <, >=, <=
	
	const String operator + (const String& rhs) const; 
	// -, *, /, % X
	
	// String *operator & () { return this; }
	// const String *operator & () const { return this; }
	
	const char *c_str() const;
	int size() const;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////
// deep copy를 사용해서 클래스를 구현해야하는 경우, 컴파일러가 제공해주는 멤버 함수는 직접 만들어야한다!
// ex) 포인터를 활용해서 객체를 복사하는 경우? - 어쨌든 shallow copy를 피해야한다!!
/////////////////////////////////////////////////////////////////////////////////////////////////////
	// String() { }
	// String(const String& rhs) { /*memberwise copy*/ }
	// ~String() { }
	
	// String& operator = (const String& rhs) { /*memeberwise copy*/ return *this; }
	
	// String *operator & () { return this; }
	// const String *operator & () const { return this; }
	
};

#endif

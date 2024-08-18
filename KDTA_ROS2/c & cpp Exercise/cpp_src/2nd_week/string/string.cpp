#include <cstring>
#include <cassert>
#include "string.h"

std::ostream& operator << (std::ostream& out, const String& rhs) {
	return out << rhs.str_;
}

String::String()
: str_(new char[1]), len_(0) {										// initialization list.
	// str_ = new char[1];
	assert(str_);
	str_[0] = '\0';
	
	// len_ = 0;
}

String::String(const char *str)
: str_(new char[strlen(str) + 1]), len_(strlen(str)) {		// initialization list.
	// str_ = new char[strlen(str) + 1];
	assert(str_);
	strcpy(str_, str);
	
	// len_ = strlen(str)
}

String::String(const String& rhs)
: str_(new char[rhs.len_ + 1]), len_(rhs.len_) {				// initialization list.
	assert(str_);
	strcpy(str_, rhs.str_);	
}

String::~String() {
	delete [] str_;
}

String& String::operator = (const String& rhs) {
	if( this != &rhs ) {									// self assignment test! (ex) s1 = s1
		// deep copy! shallow copy X
		delete [] str_;
		str_ = new char[rhs.len_ + 1];
		assert(str_);
		strcpy(str_, rhs.str_);
		len_ = rhs.len_;
	}
	return *this;
}

bool String::operator == (const String& rhs) const {
	return strcmp(str_, rhs.str_) == 0;
}

const String String::operator + (const String& rhs) const {
	char buf[strlen(str_) + rhs.len_ + 1];
	strcpy(buf, str_);
	strcat(buf, rhs.str_);
	
	String result(buf);
	
	return result;
}

const char *String::c_str() const {
	return str_;
}

int String::size() const {
	return len_;
}

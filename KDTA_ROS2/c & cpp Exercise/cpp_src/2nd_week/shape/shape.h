#ifndef SHAPE_H
#define SHAPE_H

class Shape {					// Abstract Base Class(ABC) - 추상클래스
private:
// protected:
	int x_;
	int y_;

public:
	explicit Shape(int x = 0, int y = 0);
	// Shape(const Shape& rhs);
	virtual ~Shape();
	
	// Shape& operator = (const Shape& rhs);
	void move(int x, int y);
	
	virtual double area() const = 0;					// Can't implement! // Pure Virtual func. - 순수 가상 함수(부모클래스에서 구현하지 못한 함수를 자식클래스에서 구현할 목적으로 사용. 구현 못한다는 의미로 = 0으로 정의)
	// 자식클래스에서 조차 정의 및 구현하지않으면 자식클래스 또한 추상 클래스가 된다.
	
};

#endif

#include <iostream>
#include "array.h"
#include "safeArray.h"

int main() {	
	SafeArray sArr1;
	SafeArray sArr2(10);		// SafeArray sArr2 = 10; X
	int nums[] = {1, 2, 3, 4, 5};
	SafeArray sArr3(nums, 5);
	const SafeArray sArr4 = sArr3;
	
	sArr1 = sArr3;
	
	sArr1 == sArr3;
	
	for(int i = 0; i < sArr3.size(); ++i) {
		std::cout << sArr3[i] << std::endl;
	}
	
	Array arr1(10);
	SafeArray arr2(10);
	
	// arr1[10] = 1;				// boundary error!!!
	// arr2[10] = 1;
	
	Array *pArr = new SafeArray(sArr3);			// new : 공간할당 + 생성자 호출
	(*pArr)[10] = 1;				// pArr->operator[](10) = 1;		부모 클래스쪽에 객체 호출됨.(정적(static) 바인딩)
	
	delete pArr;										// delete : 공간해제 + 소멸자 호출
	
	return 0;
}

/*
	[ 다형성(polymorphism) - (인자 전달 및 return 시 사용) ]
	 * 부모 클래스 타입의 포인터나 레퍼런스는 자식 클래스 타입의 객체를 가리키거나 참조할 수 있다.
	   (제약성 / 한정(제한)성)
*/

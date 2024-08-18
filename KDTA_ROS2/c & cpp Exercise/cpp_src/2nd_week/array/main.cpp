#include "array.h"

int main() {
	Array arr1;
	// Array arr2 = 10;		// ??? X
	Array arr2(10);

	int nums[] = {1, 2, 3, 4, 5};
	Array arr3(nums, 5);
	const Array arr4 = arr3;
	
	arr1 = arr3;
	
	if(arr1 == arr3) {
		std::cout << "equal" << std::endl;
	} else {
		std::cout << "not equal" << std::endl;
	}
	
	for(int i = 0; i < arr1.size(); ++i) {
		arr1[i] = i + 1;									// arr1.operator[](i)
	}
	
	for(int i = 0; i < arr4.size(); ++i) {
		std::cout << arr4[i] << (i != arr4.size() - 1 ? ", " : " ");
	}
	std::cout << std::endl << std::endl;
	
	std::cout << "arr1: " << arr1 << std::endl;
	std::cout << "arr2: " << arr2 << std::endl;
	std::cout << "arr3: " << arr3 << std::endl;
	std::cout << "arr4: " << arr4 << std::endl;
	
	return 0;
}

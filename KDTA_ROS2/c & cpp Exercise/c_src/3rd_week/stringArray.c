#include <stdio.h>

void printStringArray1(char (*stringArray)[20], int size) {
	for(int i = 0; i < size; ++i)
		printf("%s\n", &stringArray[i][0]);					// stringArray[i]
	printf("\n");
}

void printStringArray2(char **stringArray, int size) {		// 이중포인터는 단일 포인터를 가리킬때 사용(이중 반복문과는 상관X)
	for(int i = 0; i < size; ++i)
		printf("%s\n", stringArray[i]);						// &stringArray[i][0]
	printf("\n");
}

int main(void) {

	char cities1[][20] = {
		"Seoul", "Los Angeles", "Paris", "Rio de janeiro", "Moscow"
	};

	char *cities2[] = {
		"Seoul", "Los Angeles", "Paris", "Rio de janeiro", "Moscow"
	};
	
	printStringArray1(cities1, 5);
	printStringArray2(cities2, 5);
	
	return 0;
}

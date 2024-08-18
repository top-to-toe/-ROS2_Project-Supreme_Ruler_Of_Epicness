#include <stdio.h>
// #include <string.h>
#include "string.h"

int main(void) {

	char str1[] = "hello";
	// == char str1[] = {'h', 'e', 'l', 'l', 'o'};
	printf("str1: %s\n", str1);
	
	int len = my_strlen(str1);
	printf("len: %d\n", len);
	
	char str2[my_strlen(str1) + 1];
	my_strcpy(str2, str1);					// str2 = str1; (value copy)
	printf("str2: %s\n", str2);
	
	if(my_strcmp(str1, str2) == 0) {
		printf("str1 and str2 are equal\n");
	} else {
		printf("str1 and str2 are not equal\n");
	}
	
	char str3[] = ", world";
	printf("str3: %s\n", str3);
	char str4[my_strlen(str2) + my_strlen(str3) + 1];
	my_strcpy(str4, str2);
	my_strcat(str4, str3);
	printf("str4: %s\n", str4);
	
	len = my_strlen(str4);
	printf("len: %d\n", len);
	
	return 0;
}

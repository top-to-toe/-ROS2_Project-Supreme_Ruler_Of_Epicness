#include <stdio.h>

int main(void) {
	/*
	int i;
	i = 'A';
	
	while(i <= 'Z') {
		printf("%c\n", i);
		++i;
	}
	printf("\n");
	*/
	
	/*
	char c;
	c = (char)'A';
	
	while((int) c <= 'Z') {
		printf("%c", (int)c);
		++c;
	}
	*/
	
	for(int i = 'A'; i <= 'Z'; ++i)
		printf("%c\n", i);
	return 0;
}

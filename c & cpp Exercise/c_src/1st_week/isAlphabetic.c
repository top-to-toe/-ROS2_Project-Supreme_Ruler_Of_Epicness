#include <stdio.h>

int main(void) {
	
	char ch;
	printf("input char: ");
	scanf("%c", &ch);
	
	int isAlpha = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z');
	
	printf("%c is alphabetic : %d\n", ch, isAlpha);
	
	return 0;
}

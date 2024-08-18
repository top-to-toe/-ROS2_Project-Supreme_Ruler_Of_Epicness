#include <stdio.h>

int main(void) {

	int a, b;
	scanf("%d %d", &a, &b);
	
	int re;
	re = a > b;		printf("%d > %d : %d\n", a, b, re);
	re = a < b;		printf("%d < %d : %d\n", a, b, re);
	re = a >= b;		printf("%d >= %d : %d\n", a, b, re);
	re = a <= b;		printf("%d <= %d : %d\n", a, b, re);
	re = a == b;		printf("%d == %d : %d\n", a, b, re);
	re = a != b;		printf("%d != %d : %d\n", a, b, re);
}

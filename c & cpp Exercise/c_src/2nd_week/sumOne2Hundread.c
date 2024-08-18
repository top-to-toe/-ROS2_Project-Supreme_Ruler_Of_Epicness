#include <stdio.h>

int main(void) {
	int sum;
	int i;
	i = 1;
	while(i <= 100) {
		sum = sum + i + i + 1;
		// ++i;
		i = i + 2;
	}
	
	printf("sum[1, 100] : %d\n", sum);
	return 0;
}

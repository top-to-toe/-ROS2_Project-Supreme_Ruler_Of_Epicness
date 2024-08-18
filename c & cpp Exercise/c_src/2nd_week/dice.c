#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {
	srand(time(NULL));					// seed
	
	for (int i = 0; i <= 10; ++i) {
		int num = rand() % 6 + 1;		// random number.
		printf("%d ", num);
	}
	printf("\n");
	return 0;
}



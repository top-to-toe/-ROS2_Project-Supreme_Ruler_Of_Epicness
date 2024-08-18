#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {
	
	int balls[45];
	
	for(int i = 0; i < 45; ++i) {
		balls[i] = i + 1;
	}
	
	// shuffle & swarp
	srand(time(NULL));
	
	for(int i = 0; i <= 10000000; ++i) {
		int index1 = rand() % 45;
		int index2 = rand() % 45;
		
		int tmp = balls[index1];
		balls[index1] = balls[index2];
		balls[index2] = tmp;
	}
	
	for(int i = 0; i < 7; ++i) {
		printf("%02d ", balls[i]);
	}
	printf("\n");
	
	return 0;
}

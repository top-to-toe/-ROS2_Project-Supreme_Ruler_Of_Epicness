#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {

	int lotto[7];
	
	srand(time(NULL));
	for(int i = 0; i < 7; ) {
		// lotto[i] = [1, 45] unique!
		lotto[i] = rand() % 45 + 1;
		
		// unique???
		int j;
		for(j = 0; j < i; ++j) {
			if(lotto[i] == lotto[j]) {
				break;
			}	
		}
		if(i == j) {
			// not found (같은 값이 있을때에만 다음 위치로...)
			++i;
		} /* else {
			// found
			i = i;
		} */
	}
	
	for(int i = 0; i < 7; ++i) {
		printf("%02d ", lotto[i]);
	}
	printf("\n");
	return 0;
}

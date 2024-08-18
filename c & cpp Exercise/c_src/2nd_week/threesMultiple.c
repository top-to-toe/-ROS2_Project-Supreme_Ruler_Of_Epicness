#include <stdio.h>

int main(void) {

    int i = 1;
    while(i <= 100) {
    	if(i % 3 == 0) {
    		printf("%d --- O\n", i);
    	} else {
    		printf("%d --- X\n", i);
    	}
    // printf("%d \t %c\n", i, (i % 3 == 0) ? 'O' : 'X');
       ++i;
    }
	return 0;
}

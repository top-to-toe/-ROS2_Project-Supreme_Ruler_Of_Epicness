#include <stdio.h>
#include "serial.h"

int main(void) {
	for(int i = 1; i <= 10; ++i) {
		int num = getSerialNumber();
		fprintf(stdout, "BATTERY_%010d\n", num);
	}
	
	return 0;
}

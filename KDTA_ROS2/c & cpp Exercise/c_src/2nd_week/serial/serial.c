#include <stdio.h>
#include "serial.h"
// int count = 1;

int getSerialNumber(void) {
	// int count = 1;
	static int count = 1;
	
	return count++;
}


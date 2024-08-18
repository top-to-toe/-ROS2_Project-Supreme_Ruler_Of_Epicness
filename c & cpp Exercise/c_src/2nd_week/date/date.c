#include <stdio.h>
#include "date.h"

/*
void printDate(int year, int month, int day) {
	printf("(%d %d %d)\n", year, month, day);
}

void printDate(struct date d) {
	printf("(%d/%02d/%02d)\n", d.year, d.month, d.day);
}
*/

void printDate(const struct date *pd) {
	printf("(%d/%02d/%02d)\n", pd -> year, pd -> month, pd -> day);		// 포인터 -> 멤버
	// == printf("(%d/%02d/%02d)\n", (*pd).year, (*pd).month, (*pd).day); 같은 의미
}

#ifndef DATE_H
#define DATE_H

struct date {
	int year;
	int month;
	int day;
};

// void printDate(int year, int month, int day);
// void printDate(struct date d);
void printDate(const struct date *pd);			// 변경되지 않을 포인터가 가리키는 대상을 read only로 정의

#endif

#include <stdio.h>
#include "date.h"

int main(void) {

	// int today_year, today_month, today_day;
	struct date today;
	
	// today_year = 2024;
	// today_month = 7;
	// today_day = 12;
	today.year = 2024;
	today.month = 7;
	today.day = 12;	
	
	// int birthday_year = 1988;
	// int birthday_month = 1;
	// int birthday_day = 6;
	struct date birthday = {1988, 1, 6};
	
	// int tmp_year;
	// int tmp_month;
	// int tmp_day;
	struct date tmp;
	
	// tmp_year = today_year;
	// tmp_month = today_month;
	// tmp_day = today_day;
	tmp = today;
	
	// printDate(today_year, today_month, today_day);
	// printDate(birthday_year, birthday_month, birthday_day);
	
	/*
	printf("today: ");
	printDate(today);
	printf("tmp: ");
	printDate(tmp);
	printf("birthday: ");
	printDate(birthday);
	*/
	
	printf("today: ");
	printDate(&today);
	printf("tmp: ");
	printDate(&tmp);
	printf("birthday: ");
	printDate(&birthday);
	return 0;
}

// 구조체 자료 인자를 주소값만 전달하기위해 포인터를 사용(오버헤드를 줄이기 위해...)

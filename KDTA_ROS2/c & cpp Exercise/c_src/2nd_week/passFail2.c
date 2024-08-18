#include <stdio.h>

int main(void) {

	int score;
	printf("input score: ");
	scanf("%d", &score);
	
	/*
	if(score >= 60) {
		printf("score: %d is pass\n", score);
	} else {
		printf("score: %d is fail\n", score);
	}
	*/
	printf("score: %d is %s\n", score, (score >= 60) ? "pass" : "fail");
	
	return 0;
}

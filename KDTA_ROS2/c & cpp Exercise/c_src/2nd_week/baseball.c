#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {
	srand(time(NULL));

	int question[3];

	question[0] = rand() % 9 + 1;
	
	do {
		question[1] = rand() % 9 + 1;
	} while(question[1] == question[0]);
	
	do {
		question[2] = rand() % 9 + 1;
	} while(question[2] == question[0] || question[2] == question[1]);

	printf("question: %d %d %d\n", question[0], question[1], question[2]);
	
	int count = 0;
	while (1) {  		// infinite loop
		++count;

		int strike = 0;
		int ball = 0;

		int answer[3];
		// scanf() --> answer[]
		printf("input answer: ");
		scanf("%d %d %d", &answer[0], &answer[1], &answer[2]);
		
		for(int i = 0; i < 3; ++i) {
			for(int j = 0; j < 3; ++j) {
				if(question[i] == answer[j]) {
					if(i == j) {
						++strike;
					} else {
						++ball;
					}
				} /* else {
					None
				} */
			}
		}
		
		printf("#Count: %d,\t#Strike: %d,\t#Ball: %d\n", count, strike, ball);
		
		if (strike == 3) {
			break;
		}
	}
	return 0;
}

#include <stdio.h>

int main(void) {

	int frameScores[11] = {0};
	
	// 1 ~ 9 frame
	int scores[10][3] = {0};
	int total_score = 0;
	for(int i = 1; i <= 9; ++i) {

		do {
			 printf("%d frame 1st cast: ", i);
			 scanf("%d", &scores[i][1]);
			 if(scores[i][1] < 0 || scores[i][1] > 10) {
				  printf("Error: Score must be between 0 and 10 inclusive.\n");
			 }
		} while(scores[i][1] < 0 || scores[i][1] > 10);
		
		// case1) i-1 frame spare! --> i-1 frame score!
		if(i >= 2 && scores[i][1] + scores[i][2] == 10 && scores[i-1][1] != 10) {
			int bonus = scores[i][1];
			frameScores[i-1] = frameScores[i-2] + 10 + bonus;
			printf("%d frame score: %d\n", i-1, frameScores[i-1]);
		}
		
		// case2) i-2 frame strike, i-1 frame strike --> i-2 frame score!
		/* else */ if(i >= 3 && scores[i-2][1] == 10 && scores[i-1][1] == 10) {
			int bonus = scores[i-1][1] + scores[i][1];
			frameScores[i-2] = frameScores[i-3] + 10 + bonus;
			
			printf("%d frame score: %d\n", i-2, frameScores[i-2]);
		}

		if(scores[i][1] != 10) {
			do {
				 printf("%d frame 2nd cast: ", i);
				 scanf("%d", &scores[i][2]);
				 if(scores[i][2] < 0 || scores[i][2] > 10-scores[i][1]) {
					  printf("Error: Score must be between 0 and 10 inclusive.\n");
				 }
			} while(scores[i][2] < 0 || scores[i][2] > 10-scores[i][1]);
			
			// case1) i-1 frame strike --> i-1 frame score!
			if(i >= 2 && scores[i-1][1] == 10) {
				int bonus = scores[i-1][1] + scores[i-1][2];
				frameScores[i-1] = frameScores[i-2] + 10 + bonus;
				printf("%d frame score: %d\n", i-1, frameScores[i-1]);
			}
			
			// case2) i frame score < 10 --> i frame score!
			if(scores[i][1] + scores[i][2] < 10) {
				frameScores[i] = frameScores[i-1] + scores[i][1] + scores[i][2];
				printf("%d frame score: %d\n\n", i, frameScores[i]);
			}
		}
	}
	
	// 10th frame
	int scores10[4] = {0};
	
	// first cast.
	// frameScore -- print.
	do {
		printf("10 frame 1st cast: ");
		scanf("%d", &scores10[1]);
		if(scores10[1] < 0 || scores10[1] > 10) {
			printf("Error: Score must be between 0 and 10 inclusive.\n");
		}
	} while(scores10[1] < 0 || scores10[1] > 10);

	// case1) i-1 frame spare! --> i-1 frame score!
	if(/* i >= 2 && */ scores[9][1] + scores[9][2] == 10 && scores[9][1] != 10) {
		int bonus = scores10[1];
		frameScores[9] = frameScores[8] + 10 + bonus;
		printf("%d frame score: %d\n", 9, frameScores[9]);
	}
	// case2) i-2 frame strike, i-1 frame strike --> i-2 frame score!
	/* else */ if(/* i >= 3 && */ scores[8][1] == 10 && scores[9][1] == 10) {
		int bonus = scores[9][1] + scores10[1];
		frameScores[8] = frameScores[7] + 10 + bonus;

		printf("%d frame score: %d\n", 8, frameScores[8]);
	}
	
	// second cast.
	// frameScore -- print.
	do {
		 printf("10 frame 2nd cast: ");
		 scanf("%d", &scores10[2]);
		 if(scores10[2] < 0 || scores10[2] > 10-scores10[1]) {
			  printf("Error: Score must be between 0 and 10 inclusive.\n");
		 }
	} while(scores10[2] < 0 || scores10[2] > 10-scores10[1]);
	
	if(/* i >= 2 && */ scores[9][1] + scores[9][2] == 10 && scores[9][1] != 10) {
		int bonus = scores10[1] + scores10[2];
		frameScores[9] = frameScores[8] + 10 + bonus;
		
		printf("%d frame score: %d\n", 9, frameScores[9]);
	}
	if(scores10[1] + scores10[2] < 10) {
		frameScores[10] = frameScores[9] + scores10[1] + scores10[2];
		
		printf("%d frame score: %d\n", 10, frameScores[10]);
	} else {
		// third cast.
		printf("10 frame 3rd cast: ");
		scanf("%d", &scores10[3]);
		
		frameScores[10] = frameScores[9] + scores10[1] + scores10[2] + scores10[3];
		
		printf("%d frame score: %d\n", 10, frameScores[10]);
	}
	
	return 0;
}

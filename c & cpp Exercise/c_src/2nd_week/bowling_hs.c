#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {

	int frameScore[11] = {0};	// total frame score. all space 0 initialization
	
	// 1 ~ 9 frame
	int scores[10][3] = {0};
	
	srand(time(NULL));
	
	int frame = 1;
	while(frame <= 9) {
		printf("\nEnter score for frame %d: \n", frame);
		// scanf() --> first cast score
		/*
		do {
			printf("1st throw: ");
			scanf("%d", &scores[frame][1]);
			if(scores[frame][1] < 0 || scores[frame][1] > 10) {
				printf("Invalid input, Please enter a score between 0 and 10.\n");
			}
		} while(scores[frame][1] < 0 || scores[frame][1] > 10);
		*/
		
		scores[frame][1] = rand() % 11; // random number between 0 and 10
		printf("1st throw: %d\n", scores[frame][1]);
		
		if(scores[frame][1] == 10) {
			printf("\n\n\t\t*** Stirke! ***\n\n");
		}
		// frame score calculate...
		frameScore[frame] += scores[frame][1];
		
		if(scores[frame][1] < 10) {	//	if !Strike...
			// second cast... scanf()
			/*
			do {
				printf("2nd throw: ");
				scanf("%d", &scores[frame][2]);
				if(scores[frame][2] < 0 || scores[frame][2] > (10 - scores[frame][1])) {
					printf("Invalid input. Please enter a score between 0 and %d.\n", 10 - scores[frame][0]);
				}
			} while(scores[frame][2] < 0 || scores[frame][2] > (10 - scores[frame][1]));
			*/
			scores[frame][2] = rand() % (11 - scores[frame][1]); // random number between 0 and (10 - scores[frame][1])
         printf("2nd throw: %d\n", scores[frame][2]);
			// frame score calculate...
			frameScore[frame] += scores[frame][2];
		}
		
		
		if(frameScore[frame] == 10 && scores[frame][1] < 10) {
			
			scores[frame+1][1] = rand() % 11; // random number between 0 and 10 for bonus throw
         // printf("Bonus throw: %d\n", scores[frame+1][1]);


			frameScore[frame] += scores[frame+1][1];
		}
		// printf frameScore
		printf("\n\tFrame #%02d: %d\n", frame, frameScore[frame]);
		
		++frame;
	}
	
	// 10th frame
	int scores10[4] = {0};
	printf("\nEnter score for frame 10: \n");
	
	/* // *** 1st throw ***
	do {
		printf("1st throw: ");
		scanf("%d", &scores[10][1]);
		if(scores[10][1] < 0 || scores[10][1] > 10) {
			printf("Invalid input, Please enter a score between 0 and 10.\n");
		}
	} while(scores[10][1] < 0 || scores[10][1] > 10);
	*/
	scores[10][1] = rand() % 11; // random number between 0 and 10
	printf("1st throw: %d\n", scores[10][1]);
	if(scores[10][1] == 10) {
		printf("\n\n\t\t*** Stirke! ***\n\n");
	}
	frameScore[10] += scores[10][1];
	
	// *** 2nd throw for bonus if strike or spare ***
	if(scores[10][1] == 10 || frameScore[10] == 10) {
		/*
		do {
			printf("2nd throw: ");
			scanf("%d", &scores[10][2]);
			if(scores[10][2] < 0 || scores[10][2] > 10) {
				printf("Invalid input, Please enter a score between 0 and 10.\n");
			}
		} while(scores[10][2] < 0 || scores[10][2] > 10);
		*/
		scores[10][2] = rand() % 11; // random number between 0 and 10
		printf("2nd throw: %d\n", scores[10][2]);
		
		frameScore[10] += scores[10][2];
	}
	
	// *** 3rd throw for bonus if strike or spare ***
	if(scores[10][1] == 10 || frameScore[10] == 10) {
		/*
		do {
			printf("3rd throw: ");
			scanf("%d", &scores[10][3]);
			if(scores[10][3] < 0 || scores[10][3] > 10) {
				printf("Invalid input, Please enter a score between 0 and 10.\n");
			}
		} while(scores[10][3] < 0 || scores[10][3] > 10);
		*/
		scores[10][3] = rand() % 11; // random number between 0 and 10
		printf("3rd throw: %d\n", scores[10][3]);
		
		frameScore[10] += scores[10][3];
	}
	
	printf("\n\tFrame #10: %d\n", frameScore[10]);
	
	// Calculate and print total score
    int totalScore = 0;
    for (int i = 1; i <= 10; ++i) {
        totalScore += frameScore[i];
    }
    printf("\n\n\t\t *** Total Score: %d ***\n\n", totalScore);
	
	return 0;
}

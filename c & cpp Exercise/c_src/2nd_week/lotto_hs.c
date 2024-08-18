#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define MAX_NUM 45
#define NUM_COUNT 6

void generateLottoNumbers(int numbers[], int maxNum, int count);

int main(void) {
	int lottoNumbers[NUM_COUNT];
	srand(time(NULL));
	for(;;){
		generateLottoNumbers(lottoNumbers, MAX_NUM, NUM_COUNT);
		
		system("clear");

		printf("\n\n\n\n\n\t\t\tCreate Lotto Number: ");
		for (int i = 0; i < NUM_COUNT; i++) {
			printf("%02d ", lottoNumbers[i]);
		}
		printf("\n");
		sleep(3);
	}
	return 0;
}

void generateLottoNumbers(int numbers[], int maxNum, int count) {
	int i = 0;
	while (i < count) {
		int num = rand() % maxNum + 1;
		int duplicate = 0;
		for (int j = 0; j < i; j++) {
			if (numbers[j] == num) {
				duplicate = 1;
				break;
			}
		}
		if (!duplicate) {
			numbers[i++] = num;
		}
	}
}

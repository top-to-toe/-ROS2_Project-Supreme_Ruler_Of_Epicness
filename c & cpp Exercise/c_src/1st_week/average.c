#include <stdio.h>

int main(void) {
    int korean, english, math;
    double average;

    // 사용자로부터 국어, 영어, 수학 점수를 입력받음
    printf("Enter the Korean score: ");
    scanf("%d", &korean);
    printf("Enter the English score: ");
    scanf("%d", &english);
    printf("Enter the Math score: ");
    scanf("%d", &math);

    // 평균 계산
    average = (korean + english + math) / 3.0;

    // 계산된 평균을 출력
    printf("The average score is %.2f.\n", average);

    return 0;
}

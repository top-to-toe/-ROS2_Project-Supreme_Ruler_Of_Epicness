#include <stdio.h>

int main(void) {
	int base, height;
	double area;

    // 사용자로부터 밑변(base)과 높이(height)를 입력받음
    printf("Enter the base of the triangle: ");
    scanf("%d", &base);
    printf("Enter the height of the triangle: ");
    scanf("%d", &height);

    // 삼각형의 넓이 계산
    area = 0.5 * base * height;

    // 계산된 넓이를 출력
    printf("The area of the triangle is %.2f.\n", area);
    return 0;
}

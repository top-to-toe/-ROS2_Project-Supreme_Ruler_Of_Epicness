#include <stdio.h>
#include <stdlib.h>

int main(void) {
    int size;

    // 사용자에게 배열의 크기를 입력받기 위한 메시지 출력
    printf("input size: ");
    // 사용자로부터 정수를 입력받아 변수 size에 저장
    scanf("%d", &size);
    
    // 배열을 동적으로 생성하기 위해 메모리 할당
    // int nums[size];  // 고정 크기 배열 선언 (주석 처리됨)
    // int *pArr = malloc(sizeof(int) * size);  // 동적 메모리 할당
    // size에 따라 필요한 메모리의 크기만큼 메모리를 할당
    // malloc은 메모리 할당에 성공하면 해당 메모리의 시작 주소를 반환하고,
    // 실패하면 NULL을 반환합니다.
    int *pArr = malloc(sizeof(int) * size);

    // 동적 메모리 할당이 성공했는지 확인
    if (pArr == NULL) {
        // 메모리 할당에 실패한 경우 에러 메시지를 출력하고 프로그램 종료
        printf("Memory allocation failed\n");
        return 1;  // 비정상 종료를 나타내는 반환값
    }

    // 동적 배열에 1부터 size까지의 값을 채움
    // 반복문을 통해 배열의 각 인덱스에 i+1의 값을 저장
    for(int i = 0; i < size; ++i) {
        // nums[i] = i + 1;  // 고정 크기 배열에 값을 대입하는 방법 (주석 처리됨)
        pArr[i] = i + 1;  // 동적 배열에 값을 대입하는 방법
    }
    
    // 배열의 내용을 출력
    // 반복문을 통해 배열의 각 인덱스에 저장된 값을 출력
    for(int i = 0; i < size; ++i) {
        // printf("%d ", nums[i]);  // 고정 크기 배열의 내용을 출력하는 방법 (주석 처리됨)
        printf("%d ", pArr[i]);  // 동적 배열의 내용을 출력하는 방법
    }
    printf("\n");

    // 동적으로 할당한 메모리를 해제
    // 사용이 끝난 메모리는 반드시 해제하여 메모리 누수를 방지
    free(pArr);
    
    // 프로그램 정상 종료를 나타내는 반환값
    return 0;
}

#include <stdio.h>

int main(void) {
    int males, females;
    
    printf("Enter the number of males: ");
    scanf("%d", &males);
    
    printf("Enter the number of females: ");
    scanf("%d", &females);
    
    double males_ratio = males * 1.0 / (males + females) * 100;
    double females_ratio = females * 1.0 / (males + females) * 100;
    
    printf("Males Ratio: %.2f%%\t Females Ratio: %.2f%%\n", males_ratio, females_ratio);
    
    return 0;
}

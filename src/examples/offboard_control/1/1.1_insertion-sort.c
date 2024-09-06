#include <stdio.h>
int main(void){
    int A[] = {5, 2, 4, 6, 1, 3};
    printf("Before sorting: ");
    for (int i = 0; i < 6; i++){
        printf("%d ", A[i]);
    }
    for (int j = 1; j <= 5; j++){
        int key = A[j];
        int i = j - 1;
        while(i >= 0 && A[i] > key){
            A[i + 1] = A[i];
            i = i - 1;
        }
        A[i + 1] = key;
    }
    printf("\nAfter sorting: ");
    for (int i = 0; i < 6; i++){
        printf("%d ", A[i]);
    }
    return 0;
}
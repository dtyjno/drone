#include <stdio.h>
void merge(int A[], int p, int q, int r){
    int n1 = q - p + 1;
    int n2 = r - q;
    int L[n1 + 1], R[n2 + 1];
    for (int i = 0; i < n1; i++){
        L[i] = A[p + i];
    }
    for (int j = 0; j < n2; j++){
        R[j] = A[q + j + 1];
    }
    L[n1] = __INT_MAX__; //sentinel
    R[n2] = __INT_MAX__;
    int i = 0, j = 0;
    for (int k = p; k <= r; k++){
        if (L[i] <= R[j]){
            A[k] = L[i];
            i++;
        } else {
            A[k] = R[j];
            j++;
        }
    }
}

void merge_sort(int A[], int p, int r){
    if (p < r){
        int q = (p + r) / 2;
        merge_sort(A, p, q);
        merge_sort(A, q + 1, r);
        merge(A, p, q, r);
    }
}

int main(void){
    int A[9] = {5,6,7,9,8,4,3,2,1};
    int arraySize = sizeof(A) / sizeof(A[0]);
    merge_sort(A, 0, arraySize - 1);
    for (int i = 0; i < arraySize; i++){
        printf("%d ", A[i]);
    }
}
// Generic Matrix implementation

#ifndef MAT_H
#define MAT_H

#include <types.h>

#define MAT_SIZE 256
typedef struct {
    u16 r;
    u16 c;
    f64 data[MAT_SIZE];
} Mat;

#define MAT_AT(M, row, col) ((M).data[(row) * (M).c + (col)])

Mat mat_mul(Mat *A, Mat *B);
Mat mat_sum(Mat *A, Mat *B);
Mat mat_sub(Mat *A, Mat *B);
Mat mat_trans(Mat *M);
Mat mat_identity(u64 size);
void mat_print(Mat *M);
Mat mat_inv(Mat *M);

#endif
#include "mat.h"

#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>

Mat mat_mul(Mat *A, Mat *B) {
    assert(A->c == B->r && "Dimension mismatch");

    Mat C = {.r=A->r, .c=B->c};
    assert(C.r * C.c <= MAT_SIZE);

    for (u64 i=0; i<A->r; i++) {
        for (u64 j=0; j<B->c; j++) {
            f64 sum = 0.0;
            for (u64 k=0; k<A->c; k++) {
                sum += A->data[i*A->c + k] * B->data[k*B->c + j];
            }
            C.data[i*C.c + j] = sum;
        }
    }
    return C;
}

Mat mat_sum(Mat *A, Mat *B) {
    assert(A->r == B->r && A->c == B->c && "Dimension mismatch");
    
    Mat C = {.r=A->r, .c=A->c};
    assert(C.r * C.c <= MAT_SIZE);
    
    for (u64 i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] + B->data[i];
    }

    return C;
}

Mat mat_sub(Mat *A, Mat *B) {
    assert(A->r == B->r && A->c == B->c && "Dimension mismatch");
    
    Mat C = {.r=A->r, .c=A->c};
    assert(C.r * C.c <= MAT_SIZE);
    
    for (u64 i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] - B->data[i];
    }

    return C;
}

Mat mat_trans(Mat *M) {
    Mat N = { .r=M->c, .c=M->r };
    assert(N.r * N.c <= MAT_SIZE);
    
    for (u64 r=0; r<N.r; r++) {
        for (u64 c=0; c<N.c; c++) {
            N.data[r*N.c + c] = M->data[c*N.r + r];
        }
    }

    return N;
}

Mat mat_identity(u64 size) {
    Mat I = { .r=size, .c=size };
    assert(I.r * I.c <= MAT_SIZE);
    
    for (u64 i=0; i<size; i++) {
        I.data[size*i + i] = 1.0;
    }

    return I;
}

void mat_print(Mat *M) {
    printf("[\n");
    for (u64 r=0; r<M->r; r++) {
        printf("\t");
        for (u64 c=0; c<M->c; c++) {
            printf("%5.3lf, ", M->data[r*M->c + c]);
        }
        printf("\n");
    }
    printf("]\n");
}

Mat mat_inv(Mat *M) {
    // A * A^-1 = I
    // There's no concept of matrix division.
    // If we multiply a matrix by an inversed matrix we achieve the same results a division.

    // We're going to use Gauss-Jordan elimination with no pivoting here
    // https://en.wikipedia.org/wiki/Gaussian_elimination
    // Numerical Recipes in C, Chapter 2.1 "Gauss-Jordan Elimination"

    // Row operations:
    // * Swapping two rows,
    // * Multiplying a row by a nonzero number,
    // * Adding a multiple of one row to another row.

    assert(M->c == M->r && "The matrix must be squared");

    u64 size = M->c;

    // Build the augmented matrix
    // aug: [M | I]
    Mat aug = { .r=size, .c=2*size };
    assert(aug.r * aug.c <= MAT_SIZE);

    for (u64 r=0; r<size; r++) {
        for (u64 c=0; c<size; c++) {
            MAT_AT(aug, r, c) = MAT_AT(*M, r, c);
            //aug.data[r*aug.c + c] = M->data[r*M->c + c];
        }
    }

    for (u64 i=0; i<size; i++) {
        MAT_AT(aug, i, size+i) = 1.0;
    }
    
    //mat_print(&aug);

    for (u64 r=0; r<aug.r; r++) {
        // Normalize rows
        f64 pivot = MAT_AT(aug, r, r);

        if (fabs(pivot) < 1e-12) {
            mat_print(M);
            exit(1);
        }

        f64 col_max = 0.0;
        for (u64 rr = r; rr < size; rr++) {
            f64 v = fabs(MAT_AT(aug, rr, r));
            if (v > col_max) col_max = v;
        }

        f64 ratio = fabs(pivot) / col_max;
        if (ratio != 1.0) {
            printf("ratio: %lf\n", ratio);
            //assert(ratio == 1.0);
        }
        
        for (u64 c=r; c<aug.c; c++) {
            MAT_AT(aug, r, c) /= pivot;
        }

        // Eliminate this column from other rows
        for (u64 rr=0; rr<size; rr++) {
            if (rr == r) continue;

            f64 factor = MAT_AT(aug, rr, r); // what to subtract
            if (factor == 0.0) continue;

            for (u64 c=0; c<aug.c; c++) {
                MAT_AT(aug, rr, c) -= factor * MAT_AT(aug, r, c);
            }
        }

        // Then the right amount of the row is subtracted from each other
        // row to make all the remaining right element zero
        
        //mat_print(&aug);
    }

    Mat M_inv = {0};
    M_inv.r = size;
    M_inv.c = size;
    
    assert(M_inv.r * M_inv.c <= MAT_SIZE);

    // Copy left augmented size back
    for (u64 r=0; r<size; r++) {
        for (u64 c=0; c<size; c++) {
            MAT_AT(M_inv, r, c) = MAT_AT(aug, r, c+size);
        }
    }

    // Prove result
#ifndef NDEBUG
    Mat I_test = mat_mul(M, &M_inv);
    Mat I_known = mat_identity(size);

    for (u64 r=0; r<M->r; r++) {
        for (u64 c=0; c<M->c; c++) {
            f64 diff = MAT_AT(I_test, r, c) - MAT_AT(I_known, r, c);
            assert(fabs(diff) < 1e-14);
        }
    }
#endif

    return M_inv;
}
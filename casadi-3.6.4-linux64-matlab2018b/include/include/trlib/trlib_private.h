/* MIT License
 *
 * Copyright (c) 2016--2017 Felix Lenders
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef TRLIB_PRIVATE_H
#define TRLIB_PRIVATE_H

/* #undef TRLIB_MEASURE_TIME */
/* #undef TRLIB_MEASURE_SUBTIME */

#include "trlib.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#if defined(_WIN32) && !defined(TRLIB_BLAS_LAPACK_UNDERSCORE_SUFFIX)
    // blas
    void daxpy(trlib_int_t *n, trlib_flt_t *alpha, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    void dscal(trlib_int_t *n, trlib_flt_t *alpha, trlib_flt_t *x, trlib_int_t *incx);
    void dcopy(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    trlib_flt_t dnrm2(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx);
    trlib_flt_t ddot(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    
    // lapack
    void dpttrf(trlib_int_t *n, trlib_flt_t *d, trlib_flt_t *e, trlib_int_t *info);
    void dpttrs(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *d, trlib_flt_t *e, trlib_flt_t *b, trlib_int_t *ldb, trlib_int_t *info);
    void dptrfs(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *d, trlib_flt_t *e, trlib_flt_t *df, trlib_flt_t *ef, trlib_flt_t *b, trlib_int_t *ldb, trlib_flt_t *x, trlib_int_t *ldx, trlib_flt_t *ferr, trlib_flt_t *berr, trlib_flt_t *work, trlib_int_t *info);
    void dlagtm(char *trans, trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *alpha, trlib_flt_t *dl, trlib_flt_t *d, trlib_flt_t *du, trlib_flt_t *x, trlib_int_t *ldx, trlib_flt_t *beta, trlib_flt_t *b, trlib_int_t *ldb);
    void dgtsv(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *dl, trlib_flt_t *d, trlib_flt_t *du, trlib_flt_t *b, trlib_int_t *ldb, trlib_int_t *info);
#else
    // blas
    void daxpy_(trlib_int_t *n, trlib_flt_t *alpha, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    void dscal_(trlib_int_t *n, trlib_flt_t *alpha, trlib_flt_t *x, trlib_int_t *incx);
    void dcopy_(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    trlib_flt_t dnrm2_(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx);
    trlib_flt_t ddot_(trlib_int_t *n, trlib_flt_t *x, trlib_int_t *incx, trlib_flt_t *y, trlib_int_t *incy);
    
    // lapack
    void dpttrf_(trlib_int_t *n, trlib_flt_t *d, trlib_flt_t *e, trlib_int_t *info);
    void dpttrs_(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *d, trlib_flt_t *e, trlib_flt_t *b, trlib_int_t *ldb, trlib_int_t *info);
    void dptrfs_(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *d, trlib_flt_t *e, trlib_flt_t *df, trlib_flt_t *ef, trlib_flt_t *b, trlib_int_t *ldb, trlib_flt_t *x, trlib_int_t *ldx, trlib_flt_t *ferr, trlib_flt_t *berr, trlib_flt_t *work, trlib_int_t *info);
    void dlagtm_(char *trans, trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *alpha, trlib_flt_t *dl, trlib_flt_t *d, trlib_flt_t *du, trlib_flt_t *x, trlib_int_t *ldx, trlib_flt_t *beta, trlib_flt_t *b, trlib_int_t *ldb);
    void dgtsv_(trlib_int_t *n, trlib_int_t *nrhs, trlib_flt_t *dl, trlib_flt_t *d, trlib_flt_t *du, trlib_flt_t *b, trlib_int_t *ldb, trlib_int_t *info);
#endif

#if TRLIB_MEASURE_TIME
    #define TRLIB_TIC(X) { clock_gettime(CLOCK_MONOTONIC, &X); }
    #define TRLIB_DURATION(X, Y, Z) { clock_gettime(CLOCK_MONOTONIC, &Y); Z += 1000000000L*(Y.tv_sec-X.tv_sec)+Y.tv_nsec-X.tv_nsec; }
    #define TRLIB_SIZE_TIMING_LINALG (9)
    #if TRLIB_MEASURE_SUBTIME
        #define TRLIB_DURATION_SUB(X, Y, Z) { clock_gettime(CLOCK_MONOTONIC, &Y); Z += 1000000000L*(Y.tv_sec-X.tv_sec)+Y.tv_nsec-X.tv_nsec; }
    #else
        #define TRLIB_DURATION_SUB(X, Y, Z)
    #endif
#else
    #define TRLIB_TIC(X)
    #define TRLIB_DURATION(X, Y, Z)
    #define TRLIB_DURATION_SUB(X, Y, Z)
#endif
#define TRLIB_RETURN(X) { TRLIB_DURATION(verystart, end, timing[0]) return X; }
#if defined(_WIN32) && !defined(TRLIB_BLAS_LAPACK_UNDERSCORE_SUFFIX)
    #define TRLIB_DCOPY(...) { TRLIB_TIC(start) dcopy(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[1]) }
    #define TRLIB_DAXPY(...) { TRLIB_TIC(start) daxpy(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[2]) }
    #define TRLIB_DSCAL(...) { TRLIB_TIC(start) dscal(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[3]) }
    #define TRLIB_DNRM2(A, X, Y, Z) { TRLIB_TIC(start) A = dnrm2(X, Y, Z); TRLIB_DURATION_SUB(start, end, timing[4]) }
    #define TRLIB_DDOT(A, N, X, IX, Y, IY) { TRLIB_TIC(start) A = ddot(N, X, IX, Y, IY); TRLIB_DURATION_SUB(start, end, timing[5]) }
    #define TRLIB_DPTTRF(...) { TRLIB_TIC(start) dpttrf(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[6]) }
    #define TRLIB_DPTTRS(...) { TRLIB_TIC(start) dpttrs(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[7]) }
    #define TRLIB_DPTRFS(...) { TRLIB_TIC(start) dptrfs(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[8]) }
    #define TRLIB_DLAGTM(...) { TRLIB_TIC(start) dlagtm(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[9]) }
#else
    #define TRLIB_DCOPY(...) { TRLIB_TIC(start) dcopy_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[1]) }
    #define TRLIB_DAXPY(...) { TRLIB_TIC(start) daxpy_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[2]) }
    #define TRLIB_DSCAL(...) { TRLIB_TIC(start) dscal_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[3]) }
    #define TRLIB_DNRM2(A, X, Y, Z) { TRLIB_TIC(start) A = dnrm2_(X, Y, Z); TRLIB_DURATION_SUB(start, end, timing[4]) }
    #define TRLIB_DDOT(A, N, X, IX, Y, IY) { TRLIB_TIC(start) A = ddot_(N, X, IX, Y, IY); TRLIB_DURATION_SUB(start, end, timing[5]) }
    #define TRLIB_DPTTRF(...) { TRLIB_TIC(start) dpttrf_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[6]) }
    #define TRLIB_DPTTRS(...) { TRLIB_TIC(start) dpttrs_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[7]) }
    #define TRLIB_DPTRFS(...) { TRLIB_TIC(start) dptrfs_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[8]) }
    #define TRLIB_DLAGTM(...) { TRLIB_TIC(start) dlagtm_(__VA_ARGS__); TRLIB_DURATION_SUB(start, end, timing[9]) }
#endif
#define TRLIB_PRINTLN_1(...) if (verbose > 0) { if (fout) { fprintf(fout, "%s", prefix); fprintf(fout, __VA_ARGS__); fprintf(fout, "\n"); } else { printf("%s", prefix); printf(__VA_ARGS__); printf("\n"); } }
#define TRLIB_PRINTLN_2(...) if (verbose > 1) { if (fout) { fprintf(fout, "%s", prefix); fprintf(fout, __VA_ARGS__); fprintf(fout, "\n"); } else { printf("%s", prefix); printf(__VA_ARGS__); printf("\n"); } }

#define TRLIB_PRINT_VEC(P, N, X) { for(int vc = 0; vc < N; ++vc) { printf("%s %ld: %e\n", P, vc, *(X+vc)); } }

#endif

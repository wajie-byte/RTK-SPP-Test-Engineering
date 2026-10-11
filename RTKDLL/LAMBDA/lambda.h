#ifndef LAMBDA_H
#define LAMBDA_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>


#define MAXCOL      1024
#define LOOPMAX     5000           /* maximum count of search loop */
#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)


/* new matrix -------------------------------------------------------------*/
extern double* mat(int n, int m);
/* new integer matrix ------------------------------------------------------*/
int* imat(int n, int m);
/* zero matrix -------------------------------------------------------------*/
double* zeros(int n, int m);
/* identity matrix --------------------------------------------------------*/
double* eye(int n);
/* multiply matrix -----------------------------------------------------------*/
void matmul(const char* tr, int n, int k, int m, double alpha, const double* A, const double* B, double beta, double* C);
/* copy matrix --------------------------------------------------------------*/
void matcpy(double* A, const double* B, int n, int m);
/* solve linear equation -----------------------------------------------------*/
int solve(const char* tr, const double* A, const double* Y, int n, int m, double* X);
/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double* A, int n, int* indx, double* d);
/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double* A, int n, const int* indx, double* b);
/* inverse of matrix ---------------------------------------------------------*/
int matinv(double* A, int n);
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int LD(int n, const double* Q, double* L, double* D);  //此处分解算法同 FMFAC5（参见"模糊度经典版.pdf"）
/* LDLT factorization  ( Q=L*diag(D)*L' ) ------2017.5.9----------------------------------------- */
static int LDLT(int n, const double* Q, double* L, double* D);
/* integer gauss transformation ----------------------------------------------*/
static void gauss(int n, double* L, double* Z, int i, int j);  //类似ZTRAN算法，但i和j调换了,j表示列
/* permutations交换 --------------------------------------------------------------*/
static void perm(int n, double* L, double* D, int j, double del, double* Z);
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void reduction(int n, double* L, double* D, double* Z);
/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
static int search(int n, int m, const double* L, const double* D,
    const double* zs, double* zn, double* s);


/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions , m=2
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residuals of fixed solutions (1 x m)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
extern int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);


#endif
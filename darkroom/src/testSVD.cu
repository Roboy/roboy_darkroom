
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <iostream>

int main(void)
{
    // H has storage for 4 integers
    thrust::host_vector<int> H(4);

    // initialize individual elements
    H[0] = 14;
    H[1] = 20;
    H[2] = 38;
    H[3] = 46;

    // H.size() returns the size of vector H
    std::cout << "H has size " << H.size() << std::endl;

    // print contents of H
    for(int i = 0; i < H.size(); i++)
        std::cout << "H[" << i << "] = " << H[i] << std::endl;

    // resize H
    H.resize(2);

    std::cout << "H now has size " << H.size() << std::endl;

    // Copy host_vector H to device_vector D
    thrust::device_vector<int> D = H;

    // elements of D can be modified
    D[0] = 99;
    D[1] = 88;

    // print contents of D
    for(int i = 0; i < D.size(); i++)
        std::cout << "D[" << i << "] = " << D[i] << std::endl;

    // H and D are automatically deleted when the function returns
    return 0;
}

////Example 2. Application Using C and CUBLAS: 0-based indexing
////-----------------------------------------------------------
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <cuda_runtime.h>
//#include "cublas_v2.h"
//#define M 6
//#define N 5
//#define IDX2C(i,j,ld) (((j)*(ld))+(i))
//
//static __inline__ void modify (cublasHandle_t handle, float *m, int ldm, int n, int p, int q, float alpha, float beta){
//    cublasSscal (handle, n-p, &alpha, &m[IDX2C(p,q,ldm)], ldm);
//    cublasSscal (handle, ldm-p, &beta, &m[IDX2C(p,q,ldm)], 1);
//}
//
//int main (void){
//    cudaError_t cudaStat;
//    cublasStatus_t stat;
//    cublasHandle_t handle;
//    int i, j;
//    float* devPtrA;
//    float* a = 0;
//    a = (float *)malloc (M * N * sizeof (*a));
//    if (!a) {
//        printf ("host memory allocation failed");
//        return EXIT_FAILURE;
//    }
//    for (j = 0; j < N; j++) {
//        for (i = 0; i < M; i++) {
//            a[IDX2C(i,j,M)] = (float)(i * M + j + 1);
//        }
//    }
//    cudaStat = cudaMalloc ((void**)&devPtrA, M*N*sizeof(*a));
//    if (cudaStat != cudaSuccess) {
//        printf ("device memory allocation failed");
//        return EXIT_FAILURE;
//    }
//    stat = cublasCreate(&handle);
//    if (stat != CUBLAS_STATUS_SUCCESS) {
//        printf ("CUBLAS initialization failed\n");
//        return EXIT_FAILURE;
//    }
//    stat = cublasSetMatrix (M, N, sizeof(*a), a, M, devPtrA, M);
//    if (stat != CUBLAS_STATUS_SUCCESS) {
//        printf ("data download failed");
//        cudaFree (devPtrA);
//        cublasDestroy(handle);
//        return EXIT_FAILURE;
//    }
//    modify (handle, devPtrA, M, N, 1, 2, 16.0f, 12.0f);
//    stat = cublasGetMatrix (M, N, sizeof(*a), devPtrA, M, a, M);
//    if (stat != CUBLAS_STATUS_SUCCESS) {
//        printf ("data upload failed");
//        cudaFree (devPtrA);
//        cublasDestroy(handle);
//        return EXIT_FAILURE;
//    }
//    cudaFree (devPtrA);
//    cublasDestroy(handle);
//    for (j = 0; j < N; j++) {
//        for (i = 0; i < M; i++) {
//            printf ("%7.0f", a[IDX2C(i,j,M)]);
//        }
//        printf ("\n");
//    }
//    free(a);
//    return EXIT_SUCCESS;
//}







///*
// * How to compile (assume cuda is installed at /usr/local/cuda/)
// *   nvcc -c -I/usr/local/cuda/include svd_example.cpp
// *   g++ -fopenmp -o a.out svd_example.o -L/usr/local/cuda/lib64 -lcudart -lcublas -lcusolver
// *
// */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <assert.h>
//#include <cuda_runtime.h>
//#include <cublas_v2.h>
//#include <cusolverDn.h>
//
//void printMatrix(int m, int n, const double*A, int lda, const char* name)
//{
//    for(int row = 0 ; row < m ; row++){
//        for(int col = 0 ; col < n ; col++){
//            double Areg = A[row + col*lda];
//            printf("%s(%d,%d) = %f\n", name, row+1, col+1, Areg);
//        }
//    }
//}
//
//int main(int argc, char*argv[])
//{
//    cusolverDnHandle_t cusolverH = NULL;
//    cublasHandle_t cublasH = NULL;
//    cublasStatus_t cublas_status = CUBLAS_STATUS_SUCCESS;
//    cusolverStatus_t cusolver_status = CUSOLVER_STATUS_SUCCESS;
//    cudaError_t cudaStat1 = cudaSuccess;
//    cudaError_t cudaStat2 = cudaSuccess;
//    cudaError_t cudaStat3 = cudaSuccess;
//    cudaError_t cudaStat4 = cudaSuccess;
//    cudaError_t cudaStat5 = cudaSuccess;
//    cudaError_t cudaStat6 = cudaSuccess;
//    const int m = 3;
//    const int n = 2;
//    const int lda = m;
///*       | 1 2  |
// *   A = | 4 5  |
// *       | 2 1  |
// */
//    double A[lda*n] = { 1.0, 4.0, 2.0, 2.0, 5.0, 1.0};
//    double U[lda*m]; // m-by-m unitary matrix
//    double VT[lda*n];  // n-by-n unitary matrix
//    double S[n]; // singular value
//    double S_exact[n] = {7.065283497082729, 1.040081297712078};
//
//    double *d_A = NULL;
//    double *d_S = NULL;
//    double *d_U = NULL;
//    double *d_VT = NULL;
//    int *devInfo = NULL;
//    double *d_work = NULL;
//    double *d_rwork = NULL;
//    double *d_W = NULL;  // W = S*VT
//
//    int lwork = 0;
//    int info_gpu = 0;
//    const double h_one = 1;
//    const double h_minus_one = -1;
// printf("A = (matlab base-1)\n");
//    printMatrix(m, n, A, lda, "A");
//    printf("=====\n");
//
//// step 1: create cusolverDn/cublas handle
//    cusolver_status = cusolverDnCreate(&cusolverH);
//    assert(CUSOLVER_STATUS_SUCCESS == cusolver_status);
//
//    cublas_status = cublasCreate(&cublasH);
//    assert(CUBLAS_STATUS_SUCCESS == cublas_status);
//
//// step 2: copy A and B to device
//    cudaStat1 = cudaMalloc ((void**)&d_A  , sizeof(double)*lda*n);
//    cudaStat2 = cudaMalloc ((void**)&d_S  , sizeof(double)*n);
//    cudaStat3 = cudaMalloc ((void**)&d_U  , sizeof(double)*lda*m);
//    cudaStat4 = cudaMalloc ((void**)&d_VT , sizeof(double)*lda*n);
//    cudaStat5 = cudaMalloc ((void**)&devInfo, sizeof(int));
//    cudaStat6 = cudaMalloc ((void**)&d_W  , sizeof(double)*lda*n);
//    assert(cudaSuccess == cudaStat1);
//    assert(cudaSuccess == cudaStat2);
//    assert(cudaSuccess == cudaStat3);
//    assert(cudaSuccess == cudaStat4);
//    assert(cudaSuccess == cudaStat5);
//    assert(cudaSuccess == cudaStat6);
//
//    cudaStat1 = cudaMemcpy(d_A, A, sizeof(double)*lda*n, cudaMemcpyHostToDevice);
//    assert(cudaSuccess == cudaStat1);
//
//// step 3: query working space of SVD
//    cusolver_status = cusolverDnDgesvd_bufferSize(
//        cusolverH,
//        m,
//        n,
//        &lwork );
//    assert (cusolver_status == CUSOLVER_STATUS_SUCCESS);
//
//    cudaStat1 = cudaMalloc((void**)&d_work , sizeof(double)*lwork);
//    assert(cudaSuccess == cudaStat1);
//
//// step 4: compute SVD
//    signed char jobu = 'A'; // all m columns of U
//    signed char jobvt = 'A'; // all n columns of VT
//    cusolver_status = cusolverDnDgesvd (
//        cusolverH,
//        jobu,
//        jobvt,
//        m,
//        n,
//        d_A,
//        lda,
//        d_S,
//        d_U,
//        lda,  // ldu
//        d_VT,
//        lda, // ldvt,
//        d_work,
//        lwork,
//        d_rwork,
//        devInfo);
//    cudaStat1 = cudaDeviceSynchronize();
//    assert(CUSOLVER_STATUS_SUCCESS == cusolver_status);
//    assert(cudaSuccess == cudaStat1);
//
//      cudaStat1 = cudaMemcpy(U , d_U , sizeof(double)*lda*m, cudaMemcpyDeviceToHost);
//    cudaStat2 = cudaMemcpy(VT, d_VT, sizeof(double)*lda*n, cudaMemcpyDeviceToHost);
//    cudaStat3 = cudaMemcpy(S , d_S , sizeof(double)*n    , cudaMemcpyDeviceToHost);
//    cudaStat4 = cudaMemcpy(&info_gpu, devInfo, sizeof(int), cudaMemcpyDeviceToHost);
//    assert(cudaSuccess == cudaStat1);
//    assert(cudaSuccess == cudaStat2);
//    assert(cudaSuccess == cudaStat3);
//    assert(cudaSuccess == cudaStat4);
//
//    printf("after gesvd: info_gpu = %d\n", info_gpu);
//    assert(0 == info_gpu);
//    printf("=====\n");
//
//    printf("S = (matlab base-1)\n");
//    printMatrix(n, 1, S, lda, "S");
//    printf("=====\n");
//
//    printf("U = (matlab base-1)\n");
//    printMatrix(m, m, U, lda, "U");
//    printf("=====\n");
//
//    printf("VT = (matlab base-1)\n");
//    printMatrix(n, n, VT, lda, "VT");
//    printf("=====\n");
//
//// step 5: measure error of singular value
//    double ds_sup = 0;
//    for(int j = 0; j < n; j++){
//        double err = fabs( S[j] - S_exact[j] );
//        ds_sup = (ds_sup > err)? ds_sup : err;
//    }
//    printf("|S - S_exact| = %E \n", ds_sup);
//
//// step 6: |A - U*S*VT|
//    // W = S*VT
//    cublas_status = cublasDdgmm(
//        cublasH,
//        CUBLAS_SIDE_LEFT,
//        n,
//        n,
//        d_VT,
//        lda,
//        d_S,
//         1,
//        d_W,
//        lda);
//    assert(CUBLAS_STATUS_SUCCESS == cublas_status);
//
//      // A := -U*W + A
//    cudaStat1 = cudaMemcpy(d_A, A, sizeof(double)*lda*n, cudaMemcpyHostToDevice);
//    assert(cudaSuccess == cudaStat1);
//    cublas_status = cublasDgemm_v2(
//        cublasH,
//        CUBLAS_OP_N, // U
//        CUBLAS_OP_N, // W
//        m, // number of rows of A
//        n, // number of columns of A
//        n, // number of columns of U
//        &h_minus_one, /* host pointer */
//        d_U, // U
//        lda,
//        d_W, // W
//        lda,
//        &h_one, /* hostpointer */
//        d_A,
//        lda);
//    assert(CUBLAS_STATUS_SUCCESS == cublas_status);
//
//    double dR_fro = 0.0;
//    cublas_status = cublasDnrm2_v2(
//        cublasH, lda*n, d_A, 1, &dR_fro);
//    assert(CUBLAS_STATUS_SUCCESS == cublas_status);
//
//    printf("|A - U*S*VT| = %E \n", dR_fro);
//
//    // pinv(A) = V*S^-1*UT
//
//
//// free resources
//    if (d_A    ) cudaFree(d_A);
//    if (d_S    ) cudaFree(d_S);
//    if (d_U    ) cudaFree(d_U);
//    if (d_VT   ) cudaFree(d_VT);
//    if (devInfo) cudaFree(devInfo);
//    if (d_work ) cudaFree(d_work);
//    if (d_rwork) cudaFree(d_rwork);
//    if (d_W    ) cudaFree(d_W);
//
//    if (cublasH ) cublasDestroy(cublasH);
//    if (cusolverH) cusolverDnDestroy(cusolverH);
//
//    cudaDeviceReset();
//
//    return 0;
//}

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#define checkCudaErrors(val) check( (val), #val, __FILE__, __LINE__)

template<typename T>
void check(T err, const char* const func, const char* const file, const int line) {
  if (err != cudaSuccess) {
    std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
    std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
    exit(1);
  }
}

// checkCudaErrors(cudaMalloc(&data,    size));
// kernel launch
// cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());

#endif // UTILS_H
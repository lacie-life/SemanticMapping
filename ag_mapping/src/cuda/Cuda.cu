#include <helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace AG_MAPPING { namespace cuda {
  void deviceSynchronize() {
    checkCudaErrors( cudaDeviceSynchronize() );
  }
} }

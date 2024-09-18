#include "render.hpp"
#include <spdlog/spdlog.h>
#include <cassert>

[[gnu::noinline]]
void _abortError(const char* msg, const char* fname, int line)
{
  cudaError_t err = cudaGetLastError();
  spdlog::error("{} ({}, line: {})", msg, fname, line);
  spdlog::error("Error {}: {}", cudaGetErrorName(err), cudaGetErrorString(err));
  std::exit(1);
}

#define abortError(msg) _abortError(msg, __FUNCTION__, __LINE__)


struct rgba8_t {
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
  std::uint8_t a;
};

__device__ rgba8_t heat_lut(float x)
{
  assert(0 <= x && x <= 1);
  float x0 = 1.f / 4.f;
  float x1 = 2.f / 4.f;
  float x2 = 3.f / 4.f;

  if (x < x0)
  {
    auto g = static_cast<std::uint8_t>(x / x0 * 255);
    return rgba8_t{0, g, 255, 255};
  }
  else if (x < x1)
  {
    auto b = static_cast<std::uint8_t>((x1 - x) / x0 * 255);
    return rgba8_t{0, 255, b, 255};
  }
  else if (x < x2)
  {
    auto r = static_cast<std::uint8_t>((x - x1) / x0 * 255);
    return rgba8_t{r, 255, 0, 255};
  }
  else
  {
    auto b = static_cast<std::uint8_t>((1.f - x) / x0 * 255);
    return rgba8_t{255, b, 0, 255};
  }
}

__device__ float2 im_square(float2 z)
{
  return {z.x * z.x - z.y * z.y, 2 * z.x * z.y};
}


__device__ rgba8_t spectral_color(double l) // RGB <0,1> <- lambda l <400,700> [nm]
    {
      if (l < 0.25) {
        return {0, static_cast<std::uint8_t>(l / 0.25 * 255), 255, 255};
      } else if (l < 0.5) {
        return {0, 255, static_cast<std::uint8_t>((0.5 - l) / 0.25 * 255), 255};
      } else if (l < 0.75) {
        return {static_cast<std::uint8_t>((l - 0.5) / 0.25 * 255), 255, 0, 255};
      } else if (l < 1) {
        return {255, static_cast<std::uint8_t>((1 - l) / 0.25 * 255), 0, 255};
      } else {
        return {0, 0, 0, 255};
      }
    }


// Device code
__global__ void mykernel(char* buffer, int width, int height, size_t pitch, int n_iterations)
{
  int x = blockDim.x * blockIdx.x + threadIdx.x;
  int y = blockDim.y * blockIdx.y + threadIdx.y;

  if (x >= width || y >= height)
    return;

  uchar4*  lineptr = (uchar4*)(buffer + y * pitch);


  float2 centerPoint = {-0.75f, 0.f};
  float2 size = {1.75f, 1.f};

  float2 centered = {x - width / 2.f, y - height / 2.f};
  float2 unit = {centered.x / width * 2.f, centered.y / height * 2.f};
  float2 z = {unit.x * size.x + centerPoint.x, unit.y * size.y + centerPoint.y};

  float2 z2 = {0.f, 0.f};

  int i = 0;
  while (i < n_iterations) {
    z2 = im_square(z2);
    z2.x += z.x;
    z2.y += z.y;
    if (z2.x * z2.x + z2.y * z2.y > 4)
      break;
    i++;
  }

  float t = (float)i / n_iterations;


  // rgba8_t color = heat_lut(t);
  rgba8_t color = spectral_color(t);

  lineptr[x]    = {color.r, color.g, color.b, color.a};
  }









/// Compute the number or iteration of the fractal per pixel and store the result in *buffer*.
/// Note that a 32-bits location can be used to store an integer (int32) or a color (uchar4).
///
/// \param buffer Input buffer of type (uchar4 or uint32_t)
/// \param width Width of the image
/// \param height Height of the image
/// \param pitch Size of a line in bytes
/// \param max_iter Maximum number of iterations
__global__ void compute_iter(char* buffer, int width, int height, size_t pitch, int max_iter) {
  int x = blockDim.x * blockIdx.x + threadIdx.x;
  int y = blockDim.y * blockIdx.y + threadIdx.y;

  if (x >= width || y >= height)
    return;

  uint32_t*  lineptr = (uint32_t*)(buffer + y * pitch);


  float2 centerPoint = {-0.75f, 0.f};
  float2 size = {1.75f, 1.f};

  float2 centered = {x - width / 2.f, y - height / 2.f};
  float2 unit = {centered.x / width * 2.f, centered.y / height * 2.f};
  float2 z = {unit.x * size.x + centerPoint.x, unit.y * size.y + centerPoint.y};

  float2 z2 = {0.f, 0.f};

  int i = 0;
  while (i < max_iter) {
    z2 = im_square(z2);
    z2.x += z.x;
    z2.y += z.y;
    if (z2.x * z2.x + z2.y * z2.y > 4)
      break;
    i++;
  }

  lineptr[x] = i;
}


/// compute histo
///
/// \param buffer Input buffer of type (uchar4 or uint32_t)
/// \param width Width of the image
/// \param height Height of the image
/// \param pitch Size of a line in bytes
/// \param max_iter Maximum number of iterations
/// \param LUT Output look-up table
__global__ void compute_histo(const char* buffer, int width, int height, size_t pitch, int max_iter, uchar4* LUT) {
  int x = blockDim.x * blockIdx.x + threadIdx.x;
  int y = blockDim.y * blockIdx.y + threadIdx.y;
  if (x >= width || y >= height)
    return;


  // Compute the histogram
  uint32_t *histo = (uint32_t *) LUT;
  uint32_t *lineptr = (uint32_t*)(buffer + y * pitch);


  atomicAdd(histo + lineptr[x], 1);
}


/// This function is single thread for now!
///
/// \param width Width of the image
/// \param height Height of the image
/// \param pitch Size of a line in bytes
/// \param max_iter Maximum number of iterations
/// \param LUT Output look-up table
__global__ void compute_LUT(int width, int height, size_t pitch, int max_iter, uchar4* LUT) {
  int x = blockDim.x * blockIdx.x + threadIdx.x;
  if (x > 0)
    return;

  uint32_t *histo = (uint32_t *) LUT;


  // Compute the cumulative distribution function
  for (int i = 1; i < max_iter; i++) {
    histo[i] = histo[i - 1] + histo[i];
  }

  // rgba8_t *lut_ptr = (uchar4*)LUT;
  for (int i = 0; i < max_iter; i++) {
    rgba8_t color = heat_lut((float)histo[i] / histo[max_iter - 1]);
    LUT[i] = {color.r, color.g, color.b, 255};
  }

  LUT[max_iter] = {0, 0, 0, 255};
}


///
/// \param buffer Input buffer of type (uchar4 or uint32_t)
/// \param width Width of the image
/// \param height Height of the image
/// \param pitch Size of a line in bytes
/// \param max_iter Maximum number of iterations
__global__ void apply_LUT(char* buffer, int width, int height, size_t pitch, int max_iter, const uchar4* LUT) {
  int x = blockDim.x * blockIdx.x + threadIdx.x;
  int y = blockDim.y * blockIdx.y + threadIdx.y;

  if (x >= width || y >= height)
    return;

  uint32_t*  lineptr = (uint32_t*)(buffer + y * pitch);
  uint32_t *lut_ptr = (uint32_t*)LUT;
  lineptr[x] = lut_ptr[lineptr[x]];
}


void render(char* hostBuffer, int width, int height, std::ptrdiff_t stride, int n_iterations)
{
  cudaError_t rc = cudaSuccess;

  // Allocate device memory
  char*  devBuffer;
  size_t pitch;

  rc = cudaMallocPitch(&devBuffer, &pitch, width * sizeof(rgba8_t), height);
  if (rc)
    abortError("Fail buffer allocation");

  // Run the kernel with blocks of size 64 x 64
  {
    int bsize = 32;
    int w     = std::ceil((float)width / bsize);
    int h     = std::ceil((float)height / bsize);

    spdlog::debug("running kernel of size ({},{})", w, h);

    dim3 dimBlock(bsize, bsize);
    dim3 dimGrid(w, h);
    // mykernel<<<dimGrid, dimBlock>>>(devBuffer, width, height, pitch, n_iterations);


    uchar4* lut;
    size_t pitch_lut;
    rc = cudaMallocPitch(&lut, &pitch_lut, (n_iterations + 1) * sizeof(uchar4), 1);
    if (rc)
      abortError("Fail buffer allocation");


    compute_iter<<<dimGrid, dimBlock>>>(devBuffer, width, height, pitch, n_iterations);
    if (cudaPeekAtLastError())
      abortError("Computation Error");

    compute_histo<<<dimGrid, dimBlock>>>(devBuffer, width, height, pitch, n_iterations, lut);
    if (cudaPeekAtLastError())
      abortError("Computation Error");

    compute_LUT<<<1, 1>>>(width, height, pitch, n_iterations, lut);
    if (cudaPeekAtLastError())
      abortError("Computation Error");

    apply_LUT<<<dimGrid, dimBlock>>>(devBuffer, width, height, pitch, n_iterations, lut);
    if (cudaPeekAtLastError())
      abortError("Computation Error");
  }

  // Copy back to main memory
  rc = cudaMemcpy2D(hostBuffer, stride, devBuffer, pitch, width * sizeof(rgba8_t), height, cudaMemcpyDeviceToHost);
  if (rc)
    abortError("Unable to copy buffer back to memory");

  // Free
  rc = cudaFree(devBuffer);
  if (rc)
    abortError("Unable to free memory");
}

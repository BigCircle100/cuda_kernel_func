
#include "affine.h"
#include "utils.h"

// 求逆仿射变换矩阵
// getAffineTransform默认得到CV_64F，如果后续要float的话需要转换一下，因为存储方式不同不能直接按float转换
cv::Mat getAffineMatrix(const cv::Size& src_size, const cv::Size& dst_size){
  // int targetSize = 640;

  float scale_x = static_cast<float>(dst_size.width) / src_size.width;
  float scale_y = static_cast<float>(dst_size.height) / src_size.height;
  float scale = std::min(scale_x, scale_y);
  int dstHeight = static_cast<int>(src_size.height*scale);
  int dstWidth = static_cast<int>(src_size.width*scale);
  cv::Point2f dstCtr(dst_size.width/2.0f, dst_size.height/2.0f);

  std::vector<cv::Point2f> srcPoints = {
    cv::Point2f(0, 0),                    // left top
    cv::Point2f(src_size.width, 0),       // right top
    cv::Point2f(0, src_size.height)       // left bottom
  };
  std::vector<cv::Point2f> dstPoints = {
    cv::Point2f(dstCtr.x-dstWidth/2.0f, dstCtr.y-dstHeight/2.0f),
    cv::Point2f(dstCtr.x+dstWidth/2.0f, dstCtr.y-dstHeight/2.0f),
    cv::Point2f(dstCtr.x-dstWidth/2.0f, dstCtr.y+dstHeight/2.0f),
  };
  
  // cv::Mat M = cv::getAffineTransform(srcPoints, dstPoints);
  cv::Mat M = cv::getAffineTransform(dstPoints, srcPoints);
  cv::Mat floatMat;
  M.convertTo(floatMat, CV_32F);

  return floatMat;
}

// 这里是双线性插值（bilinar）
// 按照目标图片的点位p求原图坐标p'以及周围的四个点(v1~v4)，并根据v1~v4与p'的面积作为权重w，求出p的像素值
//    v1-------v2
//    | w4 | w3 |
//    |----p'---|
//    | w2 |  w1|
//    v3-------v4
// 每个thread负责同一位置的3通道
// image = Mat(height, width, channel)
__global__ 
void warp_affine_padding_kernel(
  uint8_t *src, int src_width, int src_height,
  uint8_t *dst, int dst_width, int dst_height,
  uint8_t c0, uint8_t c1, uint8_t c2, float *matrix){
  int idx = blockDim.x*blockIdx.x + threadIdx.x;
  int idy = blockDim.y*blockIdx.y + threadIdx.y;
  if (idx >= dst_width || idy >= dst_height){
    return;
  }

  // [x',y',1]^T = M*[x,y,1]^T 
  float src_x = matrix[0]*idx + matrix[1]*idy + matrix[2];
  float src_y = matrix[3]*idx + matrix[4]*idy + matrix[5];

  if (src_x >= 0 && src_x < src_width && src_y >= 0 && src_y < src_height){
    int x_low = floorf(src_x);
    int y_low = floorf(src_y);
    int x_high = (x_low + 1 < src_width) ? x_low+1 : x_low;
    int y_high = (y_low + 1 < src_height) ? y_low+1 : y_low;

    float ly = src_y - y_low;
    float lx = src_x - x_low;
    float hy = 1 - ly;
    float hx = 1 - lx;
    float w1 = hy * hx,  w2 = hy * lx, w3 = ly * hx, w4 = ly * lx;
    
    uint8_t* v1 = src + y_low*src_width*3 + x_low*3;
    uint8_t* v2 = src + y_low*src_width*3 + x_high*3;
    uint8_t* v3 = src + y_high*src_width*3 + x_low*3;
    uint8_t* v4 = src + y_high*src_width*3 + x_high*3;

    c0 = w1*v1[0] + w2*v2[0] + w3*v3[0] + w4*v4[0];
    c1 = w1*v1[1] + w2*v2[1] + w3*v3[1] + w4*v4[1];
    c2 = w1*v1[2] + w2*v2[2] + w3*v3[2] + w4*v4[2];

  }
  
  uint8_t* pdst = dst + idy*dst_width*3 + idx*3;
  pdst[0] = c0;
  pdst[1] = c1;
  pdst[2] = c2;


}

cv::Mat warp_affine_padding(const cv::Mat& image, const cv::Size& size){
  cv::Mat output(size, CV_8UC3);

  uint8_t* d_src;
  uint8_t* d_dst;
  float* d_matrix;

  size_t src_size = image.cols * image.rows * 3;
  size_t dst_size = size.width * size.height * 3;
  size_t matrix_size = 6 * sizeof(float);

  auto matrix = getAffineMatrix(image.size(), size);

  checkCudaErrors(cudaMalloc(&d_src, src_size));
  checkCudaErrors(cudaMalloc(&d_dst, dst_size));
  checkCudaErrors(cudaMalloc(&d_matrix, matrix_size));

  checkCudaErrors(cudaMemcpy(d_src, image.data, src_size, cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(d_matrix, matrix.ptr<float>(), matrix_size, cudaMemcpyHostToDevice));

// vvvvvvvvvvv kernel part vvvvvvvvvvv

  dim3 block_size(32, 32);
  dim3 grid_size((size.width+31)/32, (size.height+31)/32);
  warp_affine_padding_kernel<<<grid_size, block_size>>>(d_src, image.cols, image.rows, 
                                                        d_dst, size.width, size.height, 114, 114, 114, d_matrix);


// ^^^^^^^^^^^ kernel part ^^^^^^^^^^^

  cudaDeviceSynchronize(); 
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(output.data, d_dst, dst_size, cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(d_src));
  checkCudaErrors(cudaFree(d_dst));
  checkCudaErrors(cudaFree(d_matrix));

  return output;
}

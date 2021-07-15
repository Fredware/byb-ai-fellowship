/* Generated by Edge Impulse
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
// Generated on: 15.07.2021 19:46:11

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "edge-impulse-sdk/tensorflow/lite/c/builtin_op_data.h"
#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/kernels/micro_ops.h"

#if EI_CLASSIFIER_PRINT_STATE
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
    extern void ei_printf(const char *format, ...);
}
#else
extern void ei_printf(const char *format, ...);
#endif
#endif

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#endif

namespace {

constexpr int kTensorArenaSize = 160;

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX)
#pragma Bss(".tensor_arena")
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#pragma Bss()
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16) __attribute__((section(".tensor_arena")));
#else
#define EI_CLASSIFIER_ALLOCATION_HEAP 1
uint8_t* tensor_arena = NULL;
#endif

static uint8_t* tensor_boundary;
static uint8_t* current_location;

template <int SZ, class T> struct TfArray {
  int sz; T elem[SZ];
};
enum used_operators_e {
  OP_FULLY_CONNECTED, OP_SOFTMAX,  OP_LAST
};
struct TensorInfo_t { // subset of TfLiteTensor used for initialization from constant memory
  TfLiteAllocationType allocation_type;
  TfLiteType type;
  void* data;
  TfLiteIntArray* dims;
  size_t bytes;
  TfLiteQuantization quantization;
};
struct NodeInfo_t { // subset of TfLiteNode used for initialization from constant memory
  struct TfLiteIntArray* inputs;
  struct TfLiteIntArray* outputs;
  void* builtin_data;
  used_operators_e used_op_index;
};

TfLiteContext ctx{};
TfLiteTensor tflTensors[11];
TfLiteRegistration registrations[OP_LAST];
TfLiteNode tflNodes[4];

const TfArray<2, int> tensor_dimension0 = { 2, { 1,55 } };
const TfArray<1, float> quant0_scale = { 1, { 0.024192877113819122, } };
const TfArray<1, int> quant0_zero = { 1, { -86 } };
const TfLiteAffineQuantization quant0 = { (TfLiteFloatArray*)&quant0_scale, (TfLiteIntArray*)&quant0_zero, 0 };
const ALIGN(8) int32_t tensor_data1[20] = { -164, 87, 12, -210, 154, 606, 900, -328, 1222, -49, 165, 424, -47, 991, -419, 620, 1171, 184, -330, 1519, };
const TfArray<1, int> tensor_dimension1 = { 1, { 20 } };
const TfArray<1, float> quant1_scale = { 1, { 9.1897214588243514e-05, } };
const TfArray<1, int> quant1_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant1 = { (TfLiteFloatArray*)&quant1_scale, (TfLiteIntArray*)&quant1_zero, 0 };
const ALIGN(8) int32_t tensor_data2[10] = { 528, -100, -219, -420, 465, 675, -191, 185, -355, 814, };
const TfArray<1, int> tensor_dimension2 = { 1, { 10 } };
const TfArray<1, float> quant2_scale = { 1, { 0.0001602203119546175, } };
const TfArray<1, int> quant2_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant2 = { (TfLiteFloatArray*)&quant2_scale, (TfLiteIntArray*)&quant2_zero, 0 };
const ALIGN(8) int32_t tensor_data3[5] = { 229, 262, -480, -1, 254, };
const TfArray<1, int> tensor_dimension3 = { 1, { 5 } };
const TfArray<1, float> quant3_scale = { 1, { 0.0003775757213588804, } };
const TfArray<1, int> quant3_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant3 = { (TfLiteFloatArray*)&quant3_scale, (TfLiteIntArray*)&quant3_zero, 0 };
const ALIGN(8) int8_t tensor_data4[20*55] = { 
  -58, -10, -61, -34, -24, 38, -11, -63, -94, -5, 96, 13, -59, 25, 9, 100, 113, -36, 11, -61, 89, -25, 57, -100, -38, -10, -29, 21, -58, -12, -7, -27, 4, 54, 46, 5, 1, 40, 13, 15, 87, 107, 31, 24, -46, 57, 10, 16, -52, 21, 73, 83, -87, 36, 12, 
  36, 48, 5, -47, 32, 87, 66, 69, -70, 63, 48, -21, -30, -66, -30, -8, -30, -69, 41, 37, 8, -48, -73, -75, 21, -48, 9, 26, -58, -4, 50, 59, -81, -45, -78, 56, -34, 39, 27, -43, -48, 5, 12, -25, -64, -87, 14, -8, -37, -48, 23, 65, -69, -59, 59, 
  -14, -93, -62, 8, -25, 30, -38, -9, -75, -30, 6, 24, -69, -21, 24, -26, -2, -37, 73, -74, 84, 26, -46, 18, -38, 93, 36, -37, -10, -35, -10, 88, -48, 28, -23, 52, 57, -6, -34, -25, 87, 30, -15, -13, 4, 41, 76, -46, 57, -38, 101, 22, -21, -64, 52, 
  -125, -30, -71, 2, 11, -20, -56, 11, -16, 36, -34, -35, 7, 12, -18, -46, 29, -23, 58, 17, -73, 5, 64, -14, -38, 76, 56, -31, 54, 2, 20, 109, 6, -66, -13, 70, 72, -16, -63, 15, 14, 46, -18, 17, -31, -35, 116, -65, 24, 68, 71, 15, -43, 59, -57, 
  -2, 10, 6, 54, 3, 71, -62, 11, 28, 18, 35, -2, 26, -13, -68, 4, -42, -6, -1, 10, 19, -10, -3, -4, -1, -14, 82, 56, 57, 11, 20, -20, -16, 29, -23, -7, 77, 63, -66, -21, 55, -32, -17, -35, -65, 36, 74, 100, 26, 29, 82, -43, 56, -50, 54, 
  38, -38, -44, 22, 47, 8, 4, 4, 23, -14, -47, -73, 54, -9, 91, 9, -55, 19, 77, 91, -38, -78, -31, 13, -51, 25, 6, -45, 75, 11, 19, 61, -6, 108, -30, 27, -29, 22, 109, -50, -23, 2, 63, 74, -24, 25, -74, 32, 15, -5, -37, -2, -42, -1, -10, 
  -101, 56, 40, -60, -37, -21, 22, 20, -67, 24, -96, -28, -53, -46, 56, -9, 27, 53, 51, -33, -7, 34, 80, -21, 4, -23, 9, 19, 37, -25, -1, 27, -32, -15, -30, -79, 67, 62, -5, -32, -1, 59, -44, -47, -3, -57, 107, 80, 49, 50, -16, 52, 71, 3, -55, 
  -31, 62, -58, -8, -92, 82, 32, -74, 67, -84, 72, -8, -63, -39, -63, 21, 30, -59, -11, -34, 1, -35, -59, -32, -14, -37, 65, 36, 21, 57, -33, 75, -58, -36, 26, 50, -55, 13, 33, 45, 32, -51, 35, -50, -36, 79, 39, 44, 4, 8, 93, 37, -54, 49, 28, 
  49, 49, 65, -127, 41, 70, -7, 4, -19, -41, -39, 74, 11, -82, -59, -19, 14, -9, 9, 20, 20, 10, 4, -72, 1, 102, -30, 46, 54, 24, 16, 64, -5, 38, 67, 73, 32, 40, -56, 34, 96, -23, -21, 45, -13, 70, 90, 85, -54, 77, -5, -13, 69, -58, 12, 
  15, -10, 33, 48, -81, 13, -23, -12, 58, -10, 32, -52, -28, -7, 8, -1, -18, 62, 27, -47, 86, -64, -8, 29, 72, -9, -38, -39, 44, 46, -21, 12, 9, -38, -5, 98, -11, 73, 13, 59, -11, -38, 54, 53, 13, -4, 42, 16, -25, 54, 44, 92, -23, 70, 65, 
  -34, -92, -49, -23, 58, -8, -5, -4, 43, 50, 19, 10, 3, 79, -14, -61, -100, 36, -35, -9, -41, -35, 44, -67, -33, 29, -86, 49, 36, -65, 60, -38, -52, 24, 63, 19, -50, 39, -25, -5, -36, -30, 96, 90, 20, 69, -7, 90, -4, -56, 108, -32, -13, 25, 66, 
  68, 38, 29, -57, -12, 92, 18, 76, 24, 55, -5, 50, 25, -74, -57, 41, 1, 62, 40, 46, 49, -11, 20, 0, 54, -27, 10, 86, -49, 50, -36, 58, -21, -47, 56, -17, 99, 68, 39, -24, 14, 9, -36, -85, -59, 38, -44, 45, -1, 29, 33, 19, 109, 89, 101, 
  61, 47, 55, -80, 42, 7, 20, 10, -39, 55, 17, -39, -14, 49, 52, -66, 40, 37, -52, -24, -88, -16, 107, 4, -71, -34, -55, 53, 66, -39, 18, -29, 11, 46, -66, -76, -41, 22, -74, 68, -61, 9, 58, 47, -4, -85, 36, 20, -27, 41, -79, -39, 34, -71, 59, 
  0, 34, -36, 11, 23, -35, 45, -5, -19, 51, 43, 93, 58, -49, 6, -14, -2, -6, 38, 54, -104, -3, 57, 3, -86, 24, 63, 32, 108, -28, -104, 93, 39, 60, 58, -7, 78, -27, 57, -56, -76, 18, 72, -7, -18, -55, -26, 16, 15, 90, -115, -54, 15, 4, 63, 
  -25, -106, 118, 27, 21, -5, 14, 75, 69, -29, -41, -40, -49, 22, 4, -6, -57, -23, -16, -44, 24, 17, -27, 37, 48, 11, -71, 4, -12, 16, 9, -7, 67, 74, 77, -77, 18, 23, -40, -4, -10, -67, 63, -55, -24, 4, 40, -51, 47, 7, -32, 0, 66, 29, -11, 
  -50, 100, -49, -17, 16, 53, -20, -47, -61, 69, -69, 76, -22, -50, 82, -85, 46, 72, 52, 47, -16, 0, 34, -38, -48, -58, 74, 34, 26, 5, -99, -33, -58, 43, 48, -57, 1, 50, -1, 56, 18, -11, 70, -1, -14, -13, -37, 72, -18, 59, -75, 46, 26, 34, 15, 
  -77, 82, 9, 50, 23, -10, 22, -64, 20, 48, 23, -24, -59, -40, -25, 73, 88, 16, -48, 11, 5, -37, -48, 72, 71, 54, 3, -3, 81, 93, 33, 6, -17, 76, 45, 35, 17, -42, -26, 53, 43, 105, -72, -29, 62, 48, 74, 12, 84, 36, 87, 104, -57, 27, -8, 
  -73, -16, -63, -101, 50, -73, -12, 61, -73, -14, -31, 24, 17, -24, -29, -44, 15, 3, -13, 42, -93, -12, 58, -32, 46, -41, -46, 87, 61, 23, 16, 15, 36, 1, -83, -86, 63, 36, -79, 0, -79, 42, -20, 47, 67, 39, 44, 99, 19, -50, -65, 39, 5, 94, 73, 
  -51, -32, 73, 71, 23, 28, -9, -36, -16, -30, -55, -57, 64, -10, -43, 52, -53, -10, -80, -23, 49, -76, -28, 55, -56, -56, 52, -20, 0, -12, 42, 30, 60, 3, 31, 47, 46, -3, 12, 51, -25, -48, 31, -36, 73, 72, -27, 51, 32, 7, 37, -23, -33, -42, 31, 
  -13, 67, -5, 53, 61, -10, 19, -105, 75, 109, -20, 69, -33, -5, 100, 28, 43, -86, 46, 25, -22, 60, 48, -31, 80, 47, 15, 55, -36, -48, -90, -35, 61, 69, 46, -5, 63, -54, 66, -1, -73, -24, 45, 53, -22, -11, 11, 17, -8, -57, 32, 58, -53, 6, -11, 
};
const TfArray<2, int> tensor_dimension4 = { 2, { 20,55 } };
const TfArray<1, float> quant4_scale = { 1, { 0.0037985234521329403, } };
const TfArray<1, int> quant4_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant4 = { (TfLiteFloatArray*)&quant4_scale, (TfLiteIntArray*)&quant4_zero, 0 };
const ALIGN(8) int8_t tensor_data5[10*20] = { 
  -66, -11, 16, 37, 57, -46, -33, 6, -91, -66, -35, -49, 87, 89, -31, 14, -53, 20, -36, 35, 
  -60, 11, -47, 31, 22, 6, 59, -42, -32, -67, 12, -68, -23, -30, -69, -80, -40, 55, 39, -78, 
  -75, 58, 14, -54, 62, -1, 31, -44, 61, 59, -45, -42, 38, -88, 39, -31, -30, 82, -84, -31, 
  -66, -11, -56, -87, -14, 40, 60, 67, 41, -67, 66, 18, 12, -68, 17, 78, 10, 19, 17, -68, 
  15, -6, -75, -70, -99, 109, -36, -26, -65, 107, 13, -79, -54, 80, -10, 46, 24, -59, 84, 70, 
  39, 101, -45, 37, -88, -33, 27, -32, 106, -91, -118, 101, -27, 77, -82, 90, -61, 23, -100, 76, 
  97, -30, 38, 87, 96, -53, 28, 68, 92, 69, -31, 60, -95, -26, -26, -15, 51, -21, 51, -59, 
  -20, 56, -55, -99, 38, 13, -20, -43, 105, 88, 105, 101, -59, -37, 2, -37, -42, 5, -7, 32, 
  -72, 101, -1, 41, 106, 6, 91, -89, -6, -60, 52, 76, 127, 118, 17, 67, -105, 113, 6, -19, 
  -15, 31, 37, 109, -108, 64, 51, -50, 41, -9, -45, 24, -81, -1, -120, 24, 115, 90, -45, 120, 
};
const TfArray<2, int> tensor_dimension5 = { 2, { 10,20 } };
const TfArray<1, float> quant5_scale = { 1, { 0.0048567974008619785, } };
const TfArray<1, int> quant5_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant5 = { (TfLiteFloatArray*)&quant5_scale, (TfLiteIntArray*)&quant5_zero, 0 };
const ALIGN(8) int8_t tensor_data6[5*10] = { 
  -15, -8, -63, -10, -43, 94, -10, -72, 59, -21, 
  66, -86, 48, -14, 55, 16, -102, 4, -119, 43, 
  -120, 76, 56, 24, 45, -42, -29, 51, 96, -87, 
  -75, 7, 46, 19, -11, -2, 77, -126, -79, -44, 
  -31, 75, 38, 15, -89, -47, 40, 127, -33, -86, 
};
const TfArray<2, int> tensor_dimension6 = { 2, { 5,10 } };
const TfArray<1, float> quant6_scale = { 1, { 0.0066270520910620689, } };
const TfArray<1, int> quant6_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant6 = { (TfLiteFloatArray*)&quant6_scale, (TfLiteIntArray*)&quant6_zero, 0 };
const TfArray<2, int> tensor_dimension7 = { 2, { 1,20 } };
const TfArray<1, float> quant7_scale = { 1, { 0.032988879829645157, } };
const TfArray<1, int> quant7_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant7 = { (TfLiteFloatArray*)&quant7_scale, (TfLiteIntArray*)&quant7_zero, 0 };
const TfArray<2, int> tensor_dimension8 = { 2, { 1,10 } };
const TfArray<1, float> quant8_scale = { 1, { 0.05697491392493248, } };
const TfArray<1, int> quant8_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant8 = { (TfLiteFloatArray*)&quant8_scale, (TfLiteIntArray*)&quant8_zero, 0 };
const TfArray<2, int> tensor_dimension9 = { 2, { 1,5 } };
const TfArray<1, float> quant9_scale = { 1, { 0.076168432831764221, } };
const TfArray<1, int> quant9_zero = { 1, { 2 } };
const TfLiteAffineQuantization quant9 = { (TfLiteFloatArray*)&quant9_scale, (TfLiteIntArray*)&quant9_zero, 0 };
const TfArray<2, int> tensor_dimension10 = { 2, { 1,5 } };
const TfArray<1, float> quant10_scale = { 1, { 0.00390625, } };
const TfArray<1, int> quant10_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant10 = { (TfLiteFloatArray*)&quant10_scale, (TfLiteIntArray*)&quant10_zero, 0 };
const TfLiteFullyConnectedParams opdata0 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs0 = { 3, { 0,4,1 } };
const TfArray<1, int> outputs0 = { 1, { 7 } };
const TfLiteFullyConnectedParams opdata1 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs1 = { 3, { 7,5,2 } };
const TfArray<1, int> outputs1 = { 1, { 8 } };
const TfLiteFullyConnectedParams opdata2 = { kTfLiteActNone, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs2 = { 3, { 8,6,3 } };
const TfArray<1, int> outputs2 = { 1, { 9 } };
const TfLiteSoftmaxParams opdata3 = { 1 };
const TfArray<1, int> inputs3 = { 1, { 9 } };
const TfArray<1, int> outputs3 = { 1, { 10 } };
const TensorInfo_t tensorData[] = {
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension0, 55, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant0))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data1, (TfLiteIntArray*)&tensor_dimension1, 80, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant1))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data2, (TfLiteIntArray*)&tensor_dimension2, 40, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant2))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data3, (TfLiteIntArray*)&tensor_dimension3, 20, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant3))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data4, (TfLiteIntArray*)&tensor_dimension4, 1100, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant4))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data5, (TfLiteIntArray*)&tensor_dimension5, 200, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant5))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data6, (TfLiteIntArray*)&tensor_dimension6, 50, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant6))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 64, (TfLiteIntArray*)&tensor_dimension7, 20, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant7))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension8, 10, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant8))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 16, (TfLiteIntArray*)&tensor_dimension9, 5, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant9))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension10, 5, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant10))}, },
};const NodeInfo_t nodeData[] = {
  { (TfLiteIntArray*)&inputs0, (TfLiteIntArray*)&outputs0, const_cast<void*>(static_cast<const void*>(&opdata0)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs1, (TfLiteIntArray*)&outputs1, const_cast<void*>(static_cast<const void*>(&opdata1)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs2, (TfLiteIntArray*)&outputs2, const_cast<void*>(static_cast<const void*>(&opdata2)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs3, (TfLiteIntArray*)&outputs3, const_cast<void*>(static_cast<const void*>(&opdata3)), OP_SOFTMAX, },
};
static std::vector<void*> overflow_buffers;
static TfLiteStatus AllocatePersistentBuffer(struct TfLiteContext* ctx,
                                                 size_t bytes, void** ptr) {
  if (current_location - bytes < tensor_boundary) {
    // OK, this will look super weird, but.... we have CMSIS-NN buffers which
    // we cannot calculate beforehand easily.
    *ptr = malloc(bytes);
    if (*ptr == NULL) {
      printf("ERR: Failed to allocate persistent buffer of size %d\n", (int)bytes);
      return kTfLiteError;
    }
    overflow_buffers.push_back(*ptr);
    return kTfLiteOk;
  }

  current_location -= bytes;

  *ptr = current_location;
  return kTfLiteOk;
}
typedef struct {
  size_t bytes;
  void *ptr;
} scratch_buffer_t;
static std::vector<scratch_buffer_t> scratch_buffers;

static TfLiteStatus RequestScratchBufferInArena(struct TfLiteContext* ctx, size_t bytes,
                                                int* buffer_idx) {
  scratch_buffer_t b;
  b.bytes = bytes;

  TfLiteStatus s = AllocatePersistentBuffer(ctx, b.bytes, &b.ptr);
  if (s != kTfLiteOk) {
    return s;
  }

  scratch_buffers.push_back(b);

  *buffer_idx = scratch_buffers.size() - 1;

  return kTfLiteOk;
}

static void* GetScratchBuffer(struct TfLiteContext* ctx, int buffer_idx) {
  if (buffer_idx > static_cast<int>(scratch_buffers.size()) - 1) {
    return NULL;
  }
  return scratch_buffers[buffer_idx].ptr;
}
} // namespace

  TfLiteStatus trained_model_init( void*(*alloc_fnc)(size_t,size_t) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  tensor_arena = (uint8_t*) alloc_fnc(16, kTensorArenaSize);
  if (!tensor_arena) {
    printf("ERR: failed to allocate tensor arena\n");
    return kTfLiteError;
  }
#endif
  tensor_boundary = tensor_arena;
  current_location = tensor_arena + kTensorArenaSize;
  ctx.AllocatePersistentBuffer = &AllocatePersistentBuffer;
  ctx.RequestScratchBufferInArena = &RequestScratchBufferInArena;
  ctx.GetScratchBuffer = &GetScratchBuffer;
  ctx.tensors = tflTensors;
  ctx.tensors_size = 11;
  for(size_t i = 0; i < 11; ++i) {
    tflTensors[i].type = tensorData[i].type;
    tflTensors[i].is_variable = 0;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    tflTensors[i].allocation_type = tensorData[i].allocation_type;
#else
    tflTensors[i].allocation_type = (tensor_arena <= tensorData[i].data && tensorData[i].data < tensor_arena + kTensorArenaSize) ? kTfLiteArenaRw : kTfLiteMmapRo;
#endif
    tflTensors[i].bytes = tensorData[i].bytes;
    tflTensors[i].dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    if(tflTensors[i].allocation_type == kTfLiteArenaRw){
      uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

     tflTensors[i].data.data =  start;
    }
    else{
       tflTensors[i].data.data = tensorData[i].data;
    }
#else
    tflTensors[i].data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
    tflTensors[i].quantization = tensorData[i].quantization;
    if (tflTensors[i].quantization.type == kTfLiteAffineQuantization) {
      TfLiteAffineQuantization const* quant = ((TfLiteAffineQuantization const*)(tensorData[i].quantization.params));
      tflTensors[i].params.scale = quant->scale->data[0];
      tflTensors[i].params.zero_point = quant->zero_point->data[0];
    }
    if (tflTensors[i].allocation_type == kTfLiteArenaRw) {
      auto data_end_ptr = (uint8_t*)tflTensors[i].data.data + tensorData[i].bytes;
      if (data_end_ptr > tensor_boundary) {
        tensor_boundary = data_end_ptr;
      }
    }
  }
  if (tensor_boundary > current_location /* end of arena size */) {
    printf("ERR: tensor arena is too small, does not fit model - even without scratch buffers\n");
    return kTfLiteError;
  }
  registrations[OP_FULLY_CONNECTED] = *tflite::ops::micro::Register_FULLY_CONNECTED();
  registrations[OP_SOFTMAX] = *tflite::ops::micro::Register_SOFTMAX();

  for(size_t i = 0; i < 4; ++i) {
    tflNodes[i].inputs = nodeData[i].inputs;
    tflNodes[i].outputs = nodeData[i].outputs;
    tflNodes[i].builtin_data = nodeData[i].builtin_data;
    tflNodes[i].custom_initial_data = nullptr;
    tflNodes[i].custom_initial_data_size = 0;
    if (registrations[nodeData[i].used_op_index].init) {
      tflNodes[i].user_data = registrations[nodeData[i].used_op_index].init(&ctx, (const char*)tflNodes[i].builtin_data, 0);
    }
  }
  for(size_t i = 0; i < 4; ++i) {
    if (registrations[nodeData[i].used_op_index].prepare) {
      TfLiteStatus status = registrations[nodeData[i].used_op_index].prepare(&ctx, &tflNodes[i]);
      if (status != kTfLiteOk) {
        return status;
      }
    }
  }
  return kTfLiteOk;
}

static const int inTensorIndices[] = {
  0, 
};
TfLiteTensor* trained_model_input(int index) {
  return &ctx.tensors[inTensorIndices[index]];
}

static const int outTensorIndices[] = {
  10, 
};
TfLiteTensor* trained_model_output(int index) {
  return &ctx.tensors[outTensorIndices[index]];
}

TfLiteStatus trained_model_invoke() {
  for(size_t i = 0; i < 4; ++i) {
    TfLiteStatus status = registrations[nodeData[i].used_op_index].invoke(&ctx, &tflNodes[i]);

#if EI_CLASSIFIER_PRINT_STATE
    ei_printf("layer %lu\n", i);
    ei_printf("    inputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].inputs->size; ix++) {
      auto d = tensorData[tflNodes[i].inputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");

    ei_printf("    outputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].outputs->size; ix++) {
      auto d = tensorData[tflNodes[i].outputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");
#endif // EI_CLASSIFIER_PRINT_STATE

    if (status != kTfLiteOk) {
      return status;
    }
  }
  return kTfLiteOk;
}

TfLiteStatus trained_model_reset( void (*free_fnc)(void* ptr) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  free_fnc(tensor_arena);
#endif
  scratch_buffers.clear();
  for (size_t ix = 0; ix < overflow_buffers.size(); ix++) {
    free(overflow_buffers[ix]);
  }
  overflow_buffers.clear();
  return kTfLiteOk;
}

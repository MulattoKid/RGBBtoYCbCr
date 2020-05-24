/**
 * Disable warnings about using "non-safe" version of functions,
 * e.g. 'fopen' instead of 'fopen_s'. 
*/
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#define DEBUG_RGB_to_YCbCr 0
#define RGBtoYCbCr_SIMD 1
#define RGBtoYCbCr_EXECUTABLE 1

#include <math.h>
#if RGBtoYCbCr_SIMD
#include <pmmintrin.h>
#endif
#include <stdbool.h>
#include <string.h>
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "external/stb_image.h"
#if DEBUG_RGB_to_YCbCr
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image_write.h"
#endif

#include "RGBtoYCbCr.h"

// https://en.wikipedia.org/wiki/YCbCr
// https://wiki.videolan.org/YCbCr#I420
// https://rawpixels.net/
// https://software.intel.com/sites/landingpage/IntrinsicsGuide/#
/**
 * General conversion from RGB to YCbCr from Wikipedia: https://en.wikipedia.org/wiki/YCbCr
 * Y  = Kr*R + Kg*G = Kb*B
 * Pb = (B - Y) / (2 * (1 - Kb))
 * Pr = (R - Y) / (2 * (1 - Kr))
 * 
 * BT.601 (SDTV) specific:
 *  Kr = 0.299
 *  Kg = 0.587
 *  Kb = 0.114
 *  Analog YCbCr from analog RGB
 *   Y  = 0.299*R + 0.587*G + 0.114*B
 *   Pb = (B - (0.299*R + 0.587*G + 0.114*B)) / (2 * (1 - 0.114)) = (B - 0.299*R - 0.587*G - 0.114*B) / 1.772 = (-0.299*R - 0.587*G - 0.114*B + B) / 1.772
 *      = (-0.299*R - 0.587*G + B(-0.114 + 1)) / 1.772 = (-0.299*R - 0.587*G + 0.886*B) / 1.772 = -0.168735892*R - 0.331264108*G + 0.5*B
 *      = (-0.168735892*R) + (-0.331264108*G) + (0.5*B)
 *   Pr = (R - (0.299*R + 0.587*G + 0.114*B)) / (2 * (1 - 0.299)) = (R - 0.299*R - 0.587*G - 0.114*B) / 1.402 = (-0.299*R + R - 0.587*G - 0.114*B) / 1.402
 *      = (R(-0.299 + 1) - 0.587*G - 0.114*B) / 1.402  = (0.701*R - 0.587*G - 0.114*B) / 1.402  = 0.5*R - 0.418687589*G - 0.081312411*B
 *      = (0.5*R) + (-0.418687589*G) + (-0.081312411*B)
 *  Digital YCbCr from analog RGB
 *   Y  = 16 + (235-16)*Y   = 16 + 219*Y   = 16 + 219*(0.299*R + 0.587*G + 0.114*B)
 *   Cb = 128 + (240-16)*Pb = 128 + 224*Pb = 128 + 224*(-0.168735892*R - 0.331264108*G + 0.5*B)
 *   Cr = 128 + (240-16)*Pr = 128 + 224*Pr = 128 + 224*(0.5*R - 0.418687589*G - 0.081312411*B)
 *  Digital YCbCr from digital RGB (this is the one we are using)
 *   Y  = 16 + (235-16)*(Y/255) = 16 + 219*(Y/255) = 16 + 219*((0.299*R + 0.587*G + 0.114*B) / 255) = 16 + 219*((0.299*R)/255) + (0.587*G)/255 + (0.114*B)/255)
 *      = 16 + 219*((0.299/255)*R + (0.587/255)*G + (0.114/255)*B) = 16 + 219*(0.001172549*R + 0.002301961*G + 0.000447059*B)
 *      = 16 + (0.256788231*R) + (0.504129459*G) + (0.097905921*B)
 *   Cb = 128 + (240-16)*(Pb/255) = 128 + 224*(Pb/255) = 128 + 224*((-0.168735892*R - 0.331264108*G + 0.5*B) / 255) = 128 + 224*((-0.168735892*R)/255 - (0.331264108*G)/255 + (0.5*B)/255)
 *      = 128 + 224*((-0.168735892/255)*R - (0.331264108/255)*G + (0.5/255)*B) = 128 + 224*(-0.000661709*R - 0.001299075*G + 0.001960784*B)
 *      = 128 + (-0.148222816*R) + (-0.2909928*G) + (0.439215616*B)
 *   Cr = 128 + (240-16)*(Pr/255) = 128 + 224*(Pr/255) = 128 + 224*((0.5*R - 0.418687589*G - 0.081312411*B) / 255) = 128 + 224*((0.5*R)/255 - (0.418687589*G)/255 - (0.081312411*B)/255)
 *      = 128 + 224*((0.5/255)*R - (0.418687589/255)*G - (0.081312411/255)*B) = 128 + 224*(0.001960784*R - 0.001641912*G - 0.000318872*B)
 *      = 128 + (0.439215616*R) + (-0.367788288*G) + (-0.071427328*B)
 * 
 * BT.709 (HDTV) specific:
 *  Kr = 0.2126
 *  Kg = 0.7152
 *  Kb = 0.0722
 *  Analog YCbCr from analog RGB
 *   Y  = 0.2126*R + 0.7152*G + 0.0722*B
 *   Pb = (B - (0.2126*R + 0.7152*G + 0.0722*B)) / (2 * (1 - 0.0722)) = (B - 0.2126*R - 0.7152*G - 0.0722*B)) / 1.8556 = (-0.2126*R - 0.7152*G - 0.0722*B + B)) / 1.8556
 *      = (-0.2126*R - 0.7152*G + B(-0.0722 + 1)) / 1.8556 = (-0.2126*R - 0.7152*G + 0.9278*B) / 1.8556 = -0.114572106*R - 0.385427894*G + 0.5*B
 *      = (-0.114572106*R) + (-0.385427894*G) + (0.5*B)
 *   Pr = (R - (0.2126*R + 0.7152*G + 0.0722*B)) / (2 * (1 - 0.2126)) = (R - 0.2126*R - 0.7152*G - 0.0722*B) / 1.5748 = (-0.2126*R + R - 0.7152*G - 0.0722*B) / 1.5748
 *      = (R(-0.2126 + 1) - 0.7152*G - 0.0722*B) / 1.5748 = (0.7874*R - 0.7152*G - 0.0722*B) / 1.5748 = 0.5*R - 0.454152908*G - 0.045847092*B
 *      = (0.5*R) + (-0.454152908*G) + (-0.045847092*B)
 *  Digital YCbCr from analog RGB
 *   Y  = 16 + (235-16)*Y   = 16 + 219*Y  = 16 + 219*(0.2126*R + 0.7152*G + 0.0722*B)
 *   Cb = 128 + (240-16)*Pb = 128 + 224*Y = 128 + 224*(−0.114572106*R - 0.385427894*G + 0.5*B)
 *   Cr = 128 + (240-16)*Pr = 128 + 224*Y = 128 + 224*(0.5*R - 0.454152908*G - 0.045847092*B)
 *  Digital YCbCr from digital RGB
 *   Y  = 16 + (235-16)*(Y/255) = 16 + 219*(Y/255) = 16 + 219*((0.2126*R + 0.7152*G + 0.0722*B) / 255) = 16 + 219*((0.2126*R)/255 + (0.7152*G)/255 + (0.0722*B)/255)
 *      = 16 + 219*((0.2126/255)*R + (0.7152/255)*G + (0.0722/255)*B) = 16 + 219*(0.000833725*R + 0.002804706*G + 0.000283137*B)
 *      = 16 + (0.182585775*R) + (0.614230614*G) + (0.062007003*B)
 *   Cb = 128 + (240-16)*(Pb/255) = 128 + 224*(Pb/255) = 128 + 224*((−0.114572106*R - 0.385427894*G + 0.5*B) / 255) = 128 + 224*((−0.114572106*R)/255 - (0.385427894*G)/255 + (0.5*B)/255)
 *      = 128 + 224*((−0.114572106/255)*R - (0.385427894/255)*G + (0.5/255)*B) = 128 + 224*(-0.000449302*R - 0.001511482*G + 0.001960784*B)
 *      = 128 + (-0.100643648*R) + (-0.338571968*G) + (0.439215616*B)
 *   Cr = 128 + (240-16)*(Pr/255) = 128 + 224*(Pr/255) = 128 + 224*((0.5*R - 0.454152908*G - 0.045847092*B) / 255) = 128 + 224*((0.5*R)/255 - (0.454152908*G)/255 - (0.045847092*B)/255)
 *      = 128 + 224*((0.5/255)*R - (0.454152908/255)*G - (0.045847092/255)*B) = 128 + 224*(0.001960784*R - 0.001780992*G - 0.000179793*B)
 *      = 128 + (0.439215616*R) + (-0.398942208*G) + (-0.040273632*B)
 * 
 * BT.2020 (UHDTV) specific:
 *  Kr = 0.2627
 *  Kg = 0.678
 *  Kb = 0.0593
 *  Analog YCbCr from analog RGB
 *   Y  = 0.2627*R + 0.678*G + 0.0593*B
 *   Pb = (B - (0.2627*R + 0.678*G + 0.0593*B)) / (2 * (1 - 0.0593)) = (B - 0.2627*R - 0.678*G - 0.0593*B)) / 1.8814 = (-0.2627*R - 0.678*G - 0.0593*B + B)) / 1.8814
 *      = (-0.2627*R - 0.678*G + B(-0.0593 + 1)) / 1.8814 = (-0.2627*R - 0.678*G + 0.9407*B) / 1.8814 = -0.139630063*R - 0.360369937*G + 0.5*B
 *      = (-0.139630063*R) + (-0.360369937*G) + (0.5*B)
 *   Pr = (R - (0.2627*R + 0.678*G + 0.0593*B)) / (2 * (1 - 0.2627)) = (R - 0.2627*R - 0.678*G - 0.0593*B) / 1.4746 = (-0.2627*R + R - 0.678*G - 0.0593*B) / 1.4746
 *      = (R(-0.2627 + 1) - 0.678*G - 0.0593*B) / 1.4746 = (0.7373*R - 0.678*G - 0.0593*B) / 1.4746 = 0.5*R - 0.459785705*G - 0.040214295*B
 *      = (0.5*R) + (-0.459785705*G) + (-0.040214295*B)
 *  Digital YCbCr from analog RGB
 *   Y  = 16 + (235-16)*Y   = 16 + 219*(0.2627*R + 0.678*G + 0.0593*B)
 *   Cb = 128 + (240-16)*Pb = 128 + 224*(−0.139630063*R - 0.360369937*G + 0.5*B)
 *   Cr = 128 + (240-16)*Pr = 128 + 224*(0.5*R - 0.459785705*G - 0.040214295*B)
 *  Digital YCbCr from digital RGB
 *   Y  = 16 + (235-16)*(Y/255) = 16 + 219*(Y/255) = 16 + 219*((0.2627*R + 0.678*G + 0.0593*B) / 255) = 16 + 219*((0.2627*R)/255 + (0.678*G)/255 + (0.0593*B)/255)
 *      = 16 + 219*((0.2627/255)*R + (0.678/255)*G + (0.0593/255)*B) = 16 + 219*(0.001030196*R + 0.002658824*G + 0.000232549*B)
 *      = 16 + 219*(0.225612924*R) + (0.582282456*G) + (0.050928231*B)
 *   Cb = 128 + (240-16)*(Pb/255) = 128 + 224*(Pb/255) = 128 + 224*((−0.139630063*R - 0.360369937*G + 0.5*B) / 255) = 128 + 224*((−0.139630063*R)/255 - (0.360369937*G)/255 + (0.5*B)/255)
 *      = 128 + 224*((−0.139630063/255)*R - (0.360369937/255)*G + (0.5/255)*B) = 128 + 224*(-0.000547569*R - 0.001413215*G + 0.001960784*B)
 *      = 128 + (-0.122655456*R) + (-0.31656016*G) + (0.439215616*B)
 *   Cr = 128 + (240-16)*(Pr/255) = 128 + 224*(Pr/255) = 128 + 224*((0.5*R - 0.459785705*G - 0.040214295*B) / 255) = 128 + 224*((0.5*R)/255 - (0.459785705*G)/255 - (0.040214295*B)/255)
 *      = 128 + 224*((0.5/255)*R - (0.459785705/255)*G - (0.040214295/255)*B) = 128 + 224*(0.001960784*R - 0.001803081*G - 0.000157703*B)
 *      = 128 + (0.439215616*R) + (-0.403890144*G) + (-0.035325472*B)
*/

/**
 * The last element in each row is set to the value that
 * is to be added to the final result. Because the 4th value
 * in each mRGB is set to 1.0f, this value is retained, and 
 * _mm_hadd_ps can be used to sum the values in mRes at the end.
*/
static const float ENCODING_CONSTANTS[3][12] =
{
    // BT.601
    {
        0.256788231f, 0.504129459f, 0.097905921f, 16.0f,
        -0.148222816f, -0.2909928f, 0.439215616f, 128.0f,
        0.439215616f, -0.367788288f, -0.071427328f, 128.0f
    },
    // BT.709
    {
        0.182585775f, 0.614230614f, 0.062007003f, 16.0f,
        -0.100643648f, -0.338571968f, 0.439215616f, 128.0f,
        0.439215616f, -0.398942208f, -0.040273632f, 128.0f
    },
    // BT.2020
    {
        0.225612924f, 0.582282456f, 0.050928231f, 16.0f,
        -0.122655456f, -0.31656016f, 0.439215616f, 128.0f,
        0.439215616f, -0.403890144f, -0.035325472f, 128.0f
    }
};

/**
 * When adding more than two floats, the order in which they are added impacts the final result.
 * Therefore, the SIMD version will likely have minor bit-differences compared to the original method.
 * This is to be expected, and is not a bug.
 * 
 * Parameters:
 *  rgb: pointer to a 4 floats. The three first values represent the RGB components that are to be converted. The final float
 *       is set to 1.0f. This allows for the use of _mm_hadd_ps to sum the results stored in mRes.
 *  encodingConstants: pointer to 12 floats.
 *  ycbcrData: a pointer to where the converted YCbCr data should be put. For efficiency, this pointer will point to the memory
 *             address were the current RGB data recides. Since the current RGB is copied into a separate array, it is safe to
 *             override this memory.
*/
static inline void ConvertRGBtoYCbCr(const float* rgb, const float* encodingConstants, unsigned char* ycbcrData)
{
#if RGBtoYCbCr_SIMD
    float res[4];
    const __m128 mRGB = _mm_load_ps(rgb);
    __m128 mEncodingConstants = _mm_load_ps(encodingConstants);
    __m128 mRes = _mm_mul_ps(mRGB, mEncodingConstants);
    mRes = _mm_hadd_ps(mRes, mRes);
    mRes = _mm_hadd_ps(mRes, mRes);
    _mm_store_ps(res, mRes);
    const unsigned char Y  = (unsigned char)res[0];

    mEncodingConstants = _mm_load_ps(encodingConstants + 4);
    mRes = _mm_mul_ps(mRGB, mEncodingConstants);
    mRes = _mm_hadd_ps(mRes, mRes);
    mRes = _mm_hadd_ps(mRes, mRes);
    _mm_store_ps(res, mRes);
    const unsigned char Cb = (unsigned char)res[0];

    mEncodingConstants = _mm_load_ps(encodingConstants + 8);
    mRes = _mm_mul_ps(mRGB, mEncodingConstants);
    mRes = _mm_hadd_ps(mRes, mRes);
    mRes = _mm_hadd_ps(mRes, mRes);
    _mm_store_ps(res, mRes);
    const unsigned char Cr = (unsigned char)res[0];
#else
    const unsigned char Y  =
            (unsigned char)(16.0f  + (encodingConstants[0]  * rgb[0])
                                   + (encodingConstants[1]  * rgb[1])
                                   + (encodingConstants[2]  * rgb[2]));
    const unsigned char Cb =
            (unsigned char)(128.0f + (encodingConstants[4]  * rgb[0])
                                   + (encodingConstants[5]  * rgb[1])
                                   + (encodingConstants[6]  * rgb[2]));
    const unsigned char Cr =
            (unsigned char)(128.0f + (encodingConstants[8]  * rgb[0])
                                   + (encodingConstants[9]  * rgb[1])
                                   + (encodingConstants[10] * rgb[2]));
#endif

    ycbcrData[0] = Y;
    ycbcrData[1] = Cb;
    ycbcrData[2] = Cr;
}

// RGB image to YCbCr
static void ThreeComponentRGBtoYCbCr(const int imageWidth, const int imageHeight, unsigned char* rgbData, const float* encodingConstants)
{
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            const int idx = (y * imageWidth * 3) + (x * 3);
            const float r = (float)rgbData[idx + 0];
            const float g = (float)rgbData[idx + 1];
            const float b = (float)rgbData[idx + 2];
            const float rgb[4] = { r, g, b, 1.0f };
            
            ConvertRGBtoYCbCr(rgb, encodingConstants, rgbData + idx);
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the I444 format
 * I444 format:
 *  - Representation: each pixel will have its own Y, Cb and Cr values, resulting in 24 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb values; and finally all the Cr values. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCbCbCbCbCbCbCb CrCrCrCrCrCrCrCr
*/
static void YCbCrToI444(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* I444DataSize, unsigned char** I444Data)
{
    const int I444YComponentDataSize = imageWidth * imageHeight;
    const int I444CbComponentDataSize = I444YComponentDataSize;
    const int I444CrComponentDataSize = I444YComponentDataSize;
    *I444DataSize = I444YComponentDataSize + I444CbComponentDataSize + I444CrComponentDataSize;
    *I444Data = malloc(*I444DataSize);
    unsigned char* I444DataPtr = *I444Data;

    int ycbcrIdx = 0;
    int I444YComponentDataOffset = 0;
    int I444CbComponentDataOffset = I444YComponentDataSize;
    int I444CrComponentDataOffset = I444YComponentDataSize + I444CbComponentDataSize;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            I444DataPtr[I444YComponentDataOffset]  = ycbcrData[ycbcrIdx + 0];
            I444DataPtr[I444CbComponentDataOffset] = ycbcrData[ycbcrIdx + 1];
            I444DataPtr[I444CrComponentDataOffset] = ycbcrData[ycbcrIdx + 2];

            ycbcrIdx += 3;
            I444YComponentDataOffset++;
            I444CbComponentDataOffset++;
            I444CrComponentDataOffset++;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the I420 format
 * I420 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with three
 *                    other pixels, resulting in 12 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb values; and finally all the Cr values. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCb CrCr
*/
static void YCbCrToI420(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* I420DataSize, unsigned char** I420Data)
{
    const int I420YComponentDataSize = (imageWidth * imageHeight);
    const int I420CbComponentDataSize = I420YComponentDataSize / 4;
    const int I420CrComponentDataSize = I420YComponentDataSize / 4;
    *I420DataSize = I420YComponentDataSize + I420CbComponentDataSize + I420CrComponentDataSize;
    *I420Data = malloc(*I420DataSize);
    unsigned char* I420DataPtr = *I420Data;

    // Do Y component copy first
    int yIdx = 0;
    int I420Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            I420DataPtr[I420Offset] = ycbcrData[yIdx];

            yIdx += 3;
            I420Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight / 2; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int cbIdxDown = cbIdx + (imageWidth * 3);
            const int cbIdxDownRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;
            const int crIdxDown = cbIdx + (imageWidth * 3) + 1;
            const int crIdxDownRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight] + (unsigned int)ycbcrData[cbIdxDown] + (unsigned int)ycbcrData[cbIdxDownRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight] + (unsigned int)ycbcrData[crIdxDown] + (unsigned int)ycbcrData[crIdxDownRight];

            // Cb in plane 1, and Cr in plane 2
            I420DataPtr[I420Offset] = (unsigned char)(cbSum / 4);
            I420DataPtr[I420Offset + I420CbComponentDataSize] = (unsigned char)(crSum / 4);

            cbIdx += 6;
            I420Offset++;
        }
        // Skip one row
        cbIdx += (imageWidth * 3);
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the YV12 format
 * YV12 format:
 *  - Representation: (exactly like I420) each pixel will have its own Y value, and share Cb and Cr
 *                    values with three other pixels, resulting in 12 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cr values; and finally all the Cb values. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CrCr CbCb
*/
static void YCbCrToYV12(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* YV12DataSize, unsigned char** YV12Data)
{
    const int YV12YComponentDataSize = (imageWidth * imageHeight);
    const int YV12CbComponentDataSize = YV12YComponentDataSize / 4;
    const int YV12CrComponentDataSize = YV12YComponentDataSize / 4;
    *YV12DataSize = YV12YComponentDataSize + YV12CbComponentDataSize + YV12CrComponentDataSize;
    *YV12Data = malloc(*YV12DataSize);
    unsigned char* YV12DataPtr = *YV12Data;

    // Do Y component copy first
    int yIdx = 0;
    int YV12Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            YV12DataPtr[YV12Offset] = ycbcrData[yIdx];

            yIdx += 3;
            YV12Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight / 2; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int cbIdxDown = cbIdx + (imageWidth * 3);
            const int cbIdxDownRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;
            const int crIdxDown = cbIdx + (imageWidth * 3) + 1;
            const int crIdxDownRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight] + (unsigned int)ycbcrData[cbIdxDown] + (unsigned int)ycbcrData[cbIdxDownRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight] + (unsigned int)ycbcrData[crIdxDown] + (unsigned int)ycbcrData[crIdxDownRight];

            // Cr in plane 1, and Cb in plane 2
            YV12DataPtr[YV12Offset + YV12CrComponentDataSize] = (unsigned char)(cbSum / 4);
            YV12DataPtr[YV12Offset] = (unsigned char)(crSum / 4);

            cbIdx += 6;
            YV12Offset++;
        }
        // Skip one row
        cbIdx += (imageWidth * 3);
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the I422 format
 * I422 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr
 *                    values with one other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb values; and finally all the Cr values. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCbCbCb CrCrCrCr
*/
static void YCbCrToI422(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* I422DataSize, unsigned char** I422Data)
{
    const int I422YComponentDataSize = (imageWidth * imageHeight);
    const int I422CbComponentDataSize = I422YComponentDataSize / 2;
    const int I422CrComponentDataSize = I422YComponentDataSize / 2;
    *I422DataSize = I422YComponentDataSize + I422CbComponentDataSize + I422CrComponentDataSize;
    *I422Data = malloc(*I422DataSize);
    unsigned char* I422DataPtr = *I422Data;

    // Do Y component copy first
    int yIdx = 0;
    int I422Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            I422DataPtr[I422Offset] = ycbcrData[yIdx];

            yIdx += 3;
            I422Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdxRight + 1;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            // Cr in plane 1, and Cb in plane 2
            I422DataPtr[I422Offset] = (unsigned char)(cbSum / 2);
            I422DataPtr[I422Offset + I422CbComponentDataSize] = (unsigned char)(crSum / 2);

            cbIdx += 6;
            I422Offset++;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV12 format
 * NV12 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with three
 *                    other pixels, resulting in 12 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cb coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCrCbCr
*/
static void YCbCrToNV12(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV12DataSize, unsigned char** NV12Data)
{
    const int NV12YComponentDataSize = (imageWidth * imageHeight);
    const int NV12CbComponentDataSize = NV12YComponentDataSize / 4;
    const int NV12CrComponentDataSize = NV12YComponentDataSize / 4;
    *NV12DataSize = NV12YComponentDataSize + NV12CbComponentDataSize + NV12CrComponentDataSize;
    *NV12Data = malloc(*NV12DataSize);
    unsigned char* NV12DataPtr = *NV12Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV12Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV12DataPtr[NV12Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV12Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight / 2; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int cbIdxDown = cbIdx + (imageWidth * 3);
            const int cbIdxDownRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;
            const int crIdxDown = cbIdx + (imageWidth * 3) + 1;
            const int crIdxDownRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight] + (unsigned int)ycbcrData[cbIdxDown] + (unsigned int)ycbcrData[cbIdxDownRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight] + (unsigned int)ycbcrData[crIdxDown] + (unsigned int)ycbcrData[crIdxDownRight];

            // Cb and Cr are interleaved in the same plane
            NV12DataPtr[NV12Offset] = (unsigned char)(cbSum / 4);
            NV12DataPtr[NV12Offset + 1] = (unsigned char)(crSum / 4);

            cbIdx += 6;
            NV12Offset += 2;
        }
        // Skip one row
        cbIdx += (imageWidth * 3);
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV12 format
 * NV12 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with three
 *                    other pixels, resulting in 12 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cr coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CrCbCrCb
*/
static void YCbCrToNV21(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV21DataSize, unsigned char** NV21Data)
{
    const int NV21YComponentDataSize = (imageWidth * imageHeight);
    const int NV21CbComponentDataSize = NV21YComponentDataSize / 4;
    const int NV21CrComponentDataSize = NV21YComponentDataSize / 4;
    *NV21DataSize = NV21YComponentDataSize + NV21CbComponentDataSize + NV21CrComponentDataSize;
    *NV21Data = malloc(*NV21DataSize);
    unsigned char* NV21DataPtr = *NV21Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV21Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV21DataPtr[NV21Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV21Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight / 2; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int cbIdxDown = cbIdx + (imageWidth * 3);
            const int cbIdxDownRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;
            const int crIdxDown = cbIdx + (imageWidth * 3) + 1;
            const int crIdxDownRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight] + (unsigned int)ycbcrData[cbIdxDown] + (unsigned int)ycbcrData[cbIdxDownRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight] + (unsigned int)ycbcrData[crIdxDown] + (unsigned int)ycbcrData[crIdxDownRight];

            // Cr and Cb are interleaved in the same plane
            NV21DataPtr[NV21Offset] = (unsigned char)(crSum / 4);
            NV21DataPtr[NV21Offset + 1] = (unsigned char)(cbSum / 4);

            cbIdx += 6;
            NV21Offset += 2;
        }
        // Skip one row
        cbIdx += (imageWidth * 3);
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV16 format
 * NV16 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with one
 *                    other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cb coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCrCbCrCbCrCbCr
*/
static void YCbCrToNV16(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV16DataSize, unsigned char** NV16Data)
{
    const int NV16YComponentDataSize = (imageWidth * imageHeight);
    const int NV16CbComponentDataSize = NV16YComponentDataSize / 2;
    const int NV16CrComponentDataSize = NV16YComponentDataSize / 2;
    *NV16DataSize = NV16YComponentDataSize + NV16CbComponentDataSize + NV16CrComponentDataSize;
    *NV16Data = malloc(*NV16DataSize);
    unsigned char* NV16DataPtr = *NV16Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV16Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV16DataPtr[NV16Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV16Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            // Cr and Cb are interleaved in the same plane
            NV16DataPtr[NV16Offset] = (unsigned char)(cbSum / 2);
            NV16DataPtr[NV16Offset + 1] = (unsigned char)(crSum / 2);

            cbIdx += 6;
            NV16Offset += 2;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV61 format
 * NV61 format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with one
 *                    other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cr coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCrCbCrCbCrCbCr
*/
static void YCbCrToNV61(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV61DataSize, unsigned char** NV61Data)
{
    const int NV61YComponentDataSize = (imageWidth * imageHeight);
    const int NV61CbComponentDataSize = NV61YComponentDataSize / 2;
    const int NV61CrComponentDataSize = NV61YComponentDataSize / 2;
    *NV61DataSize = NV61YComponentDataSize + NV61CbComponentDataSize + NV61CrComponentDataSize;
    *NV61Data = malloc(*NV61DataSize);
    unsigned char* NV61DataPtr = *NV61Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV61Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV61DataPtr[NV61Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV61Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdxRight = cbIdx + 3;
            const int crIdx = cbIdx + 1;
            const int crIdxRight = cbIdx + 4;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            // Cr and Cb are interleaved in the same plane
            NV61DataPtr[NV61Offset] = (unsigned char)(crSum / 2);
            NV61DataPtr[NV61Offset + 1] = (unsigned char)(cbSum / 2);

            cbIdx += 6;
            NV61Offset += 2;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV24 format
 * NV24 format:
 *  - Representation: each pixel will have its own Y, Cb and Cr values, resulting in 24 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cb coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CbCrCbCrCbCrCbCrCbCrCbCrCbCrCbCr
*/
static void YCbCrToNV24(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV24DataSize, unsigned char** NV24Data)
{
    const int NV24YComponentDataSize = (imageWidth * imageHeight);
    const int NV24CbComponentDataSize = NV24YComponentDataSize;
    const int NV24CrComponentDataSize = NV24YComponentDataSize;
    *NV24DataSize = NV24YComponentDataSize + NV24CbComponentDataSize + NV24CrComponentDataSize;
    *NV24Data = malloc(*NV24DataSize);
    unsigned char* NV24DataPtr = *NV24Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV24Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV24DataPtr[NV24Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV24Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            const int crIdx = cbIdx + 1;

            // Cr and Cb are interleaved in the same plane
            NV24DataPtr[NV24Offset] = ycbcrData[cbIdx];
            NV24DataPtr[NV24Offset + 1] = ycbcrData[crIdx];

            cbIdx += 3;
            NV24Offset += 2;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the NV42 format
 * NV42 format:
 *  - Representation: each pixel will have its own Y, Cb and Cr values, resulting in 24 bits being needed per pixel.
 *  - Layout: all Y values come first; then all the Cb and Cr values interleaved, with Cr coming first. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YYYYYYYY CrCbCrCbCrCbCrCbCrCbCrCbCrCbCrCb
*/
static void YCbCrToNV42(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* NV42DataSize, unsigned char** NV42Data)
{
    const int NV42YComponentDataSize = (imageWidth * imageHeight);
    const int NV42CbComponentDataSize = NV42YComponentDataSize;
    const int NV42CrComponentDataSize = NV42YComponentDataSize;
    *NV42DataSize = NV42YComponentDataSize + NV42CbComponentDataSize + NV42CrComponentDataSize;
    *NV42Data = malloc(*NV42DataSize);
    unsigned char* NV42DataPtr = *NV42Data;

    // Do Y component copy first
    int yIdx = 0;
    int NV42Offset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            NV42DataPtr[NV42Offset] = ycbcrData[yIdx];

            yIdx += 3;
            NV42Offset++;
        }
    }

    // Do Cb and Cr in same loop
    int cbIdx = 1;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth; x++)
        {
            const int crIdx = cbIdx + 1;

            // Cr and Cb are interleaved in the same plane
            NV42DataPtr[NV42Offset] = ycbcrData[crIdx];
            NV42DataPtr[NV42Offset + 1] = ycbcrData[cbIdx];

            cbIdx += 3;
            NV42Offset += 2;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the UYVY format
 * UYVY format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with one
 *                    other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: the Cb value comes first, followed by the first Y value, then the Cr value, and finally the last Y value. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              CbYCrY CbYCrY CbYCrY CbYCrY
*/
static void YCbCrToUYVY(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* UYVYDataSize, unsigned char** UYVYData)
{
    const int UYVYYComponentDataSize = (imageWidth * imageHeight);
    const int UYVYCbComponentDataSize = UYVYYComponentDataSize / 2;
    const int UYVYCrComponentDataSize = UYVYYComponentDataSize / 2;
    *UYVYDataSize = UYVYYComponentDataSize + UYVYCbComponentDataSize + UYVYCrComponentDataSize;
    *UYVYData = malloc(*UYVYDataSize);
    unsigned char* UYVYDataPtr = *UYVYData;

    // Do all components in same loop
    int idx = 0;
    int UYVYOffset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdx = idx + 1;
            const int crIdx = idx + 2;
            const int yIdxRight = idx + 3;
            const int cbIdxRight = idx + 4;
            const int crIdxRight = idx + 5;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            UYVYDataPtr[UYVYOffset + 0] = (unsigned char)(cbSum / 2);
            UYVYDataPtr[UYVYOffset + 1] = ycbcrData[idx];
            UYVYDataPtr[UYVYOffset + 2] = (unsigned char)(crSum / 2);
            UYVYDataPtr[UYVYOffset + 3] = ycbcrData[yIdxRight];

            idx += 6;
            UYVYOffset += 4;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the YUYV format
 * YUYV format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with one
 *                    other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: the first Y value comes first, followed by the the Cb value, then the second Y value,
 *            and finally the Cr value. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YCbYCr YCbYCr YCbYCr YCbYCr
*/
static void YCbCrToYUYV(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* YUYVDataSize, unsigned char** YUYVData)
{
    const int YUYVYComponentDataSize = (imageWidth * imageHeight);
    const int YUYVCbComponentDataSize = YUYVYComponentDataSize / 2;
    const int YUYVCrComponentDataSize = YUYVYComponentDataSize / 2;
    *YUYVDataSize = YUYVYComponentDataSize + YUYVCbComponentDataSize + YUYVCrComponentDataSize;
    *YUYVData = malloc(*YUYVDataSize);
    unsigned char* YUYVDataPtr = *YUYVData;

    // Do all components in same loop
    int idx = 0;
    int YUYVOffset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdx = idx + 1;
            const int crIdx = idx + 2;
            const int yIdxRight = idx + 3;
            const int cbIdxRight = idx + 4;
            const int crIdxRight = idx + 5;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            YUYVDataPtr[YUYVOffset + 0] = ycbcrData[idx];
            YUYVDataPtr[YUYVOffset + 1] = (unsigned char)(cbSum / 2);
            YUYVDataPtr[YUYVOffset + 2] = ycbcrData[yIdxRight];
            YUYVDataPtr[YUYVOffset + 3] = (unsigned char)(crSum / 2);

            idx += 6;
            YUYVOffset += 4;
        }
    }
}

/**
 * Convert YCbCr data (that was converted directly from RGB) to the YVYU format
 * YVYU format:
 *  - Representation: each pixel will have its own Y value, and share Cb and Cr values with one
 *                    other pixel, resulting in 16 bits being needed per pixel.
 *  - Layout: the first Y value comes first, followed by the the Cr value, then the second Y value,
 *            and finally the Cb value. I.e.
 *              YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr YCbCr
 *            becomes
 *              YCrYCb YCrYCb YCrYCb YCrYCb
*/
static void YCbCrToYVYU(const int imageWidth, const int imageHeight, const unsigned char* ycbcrData, int* YVYUDataSize, unsigned char** YVYUData)
{
    const int YVYUYComponentDataSize = (imageWidth * imageHeight);
    const int YVYUCbComponentDataSize = YVYUYComponentDataSize / 2;
    const int YVYUCrComponentDataSize = YVYUYComponentDataSize / 2;
    *YVYUDataSize = YVYUYComponentDataSize + YVYUCbComponentDataSize + YVYUCrComponentDataSize;
    *YVYUData = malloc(*YVYUDataSize);
    unsigned char* YVYUDataPtr = *YVYUData;

    // Do all components in same loop
    int idx = 0;
    int YVYUOffset = 0;
    for (int y = 0; y < imageHeight; y++)
    {
        for (int x = 0; x < imageWidth / 2; x++)
        {
            const int cbIdx = idx + 1;
            const int crIdx = idx + 2;
            const int yIdxRight = idx + 3;
            const int cbIdxRight = idx + 4;
            const int crIdxRight = idx + 5;

            const unsigned int cbSum = (unsigned int)ycbcrData[cbIdx] + (unsigned int)ycbcrData[cbIdxRight];
            const unsigned int crSum = (unsigned int)ycbcrData[crIdx] + (unsigned int)ycbcrData[crIdxRight];

            YVYUDataPtr[YVYUOffset + 0] = ycbcrData[idx];
            YVYUDataPtr[YVYUOffset + 1] = (unsigned char)(crSum / 2);
            YVYUDataPtr[YVYUOffset + 2] = ycbcrData[yIdxRight];
            YVYUDataPtr[YVYUOffset + 3] = (unsigned char)(cbSum / 2);

            idx += 6;
            YVYUOffset += 4;
        }
    }
}

int RGBtoYCbCr(const char* inputFile, YCbCrEncoding outputEncoding, YCbCrFormat outputFormat, const char* outputFile)
{
    // Load RGB data using stb - force 3 components
    clock_t imageLoadStartTime = clock();
    int imageWidth, imageHeight, imageComponents;
    unsigned char* rgbData = stbi_load(inputFile, &imageWidth, &imageHeight, &imageComponents, 3);
    if (rgbData == NULL)
    {
        printf("ERROR: Failed to load input image: %s\n", inputFile);
        return -1;
    }
    printf("INFO: Image file: %s\n", inputFile);
    printf("INFO: Image dimensions: %ix%i\n", imageWidth, imageHeight);
    printf("INFO: Image components: %i\n", imageComponents);
    if (imageWidth % 2 != 0)
    {
        printf("ERROR: Invalid image width %i: must be divisble by 2\n", imageWidth);
        stbi_image_free(rgbData);
        return -1;
    }
    if (imageHeight % 2 != 0)
    {
        printf("ERROR: Invalid image height %i: must be divisble by 2\n", imageHeight);
        stbi_image_free(rgbData);
        return -1;
    }
    clock_t imageLoadEndTime = clock();

    // Convert all RGB data to YCbCr
    clock_t conversionStartTime = clock();
    const float* encodingConstants = ENCODING_CONSTANTS[outputEncoding];
    switch (imageComponents)
    {
        case 3:
        {
            ThreeComponentRGBtoYCbCr(imageWidth, imageHeight, rgbData, encodingConstants);
            break;
        }
        default:
        {
            printf("ERROR: Invalid number of components in RGB image: %i\n", imageComponents);
            stbi_image_free(rgbData);
            return -1;
        }
    }
    // YCbCrDataSize = imageWidth * imageHeight * 3;
    unsigned char* ycbcrData = rgbData;
    clock_t conversionEndTime = clock();

    // Compress YCbCr data to given output format
    clock_t compressionStartTime = clock();
    int outputYcbcrDataSize;
    unsigned char* outputYcbcrData;
    switch (outputFormat)
    {
        case I444:
        {
            YCbCrToI444(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case I420:
        {
            YCbCrToI420(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case YV12:
        {
            YCbCrToYV12(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case I422:
        {
            YCbCrToI422(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV12:
        {
            YCbCrToNV12(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV21:
        {
            YCbCrToNV21(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV16:
        {
            YCbCrToNV16(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV61:
        {
            YCbCrToNV61(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV24:
        {
            YCbCrToNV24(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case NV42:
        {
            YCbCrToNV42(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case UYVY:
        {
            YCbCrToUYVY(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case Y422:
        {
            YCbCrToUYVY(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case UYNV:
        {
            YCbCrToUYVY(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case YUYV:
        {
            YCbCrToYUYV(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case V422:
        {
            YCbCrToYUYV(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case YUNV:
        {
            YCbCrToYUYV(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case YUY2:
        {
            YCbCrToYUYV(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        case YVYU:
        {
            YCbCrToYVYU(imageWidth, imageHeight, ycbcrData, &outputYcbcrDataSize, &outputYcbcrData);
            break;
        }
        default:
        {
            printf("ERROR: Invalid YCbCr output format: %i\n", outputFormat);
            stbi_image_free(rgbData);
            return -1;
        }
    }
    clock_t compressionEndTime = clock();

    // Write out final YCbCr data
    clock_t writeOutStartTime = clock();
    FILE* ycbcrFile = fopen(outputFile, "wb");
    if (ycbcrFile == NULL)
    {
        printf("ERROR: Failed to open output file: %s\n", outputFile);
        free(outputYcbcrData);
        stbi_image_free(rgbData);
        return -1;
    }
    const size_t numElementsWritten = fwrite(outputYcbcrData, 1, outputYcbcrDataSize, ycbcrFile);
    if (numElementsWritten != outputYcbcrDataSize)
    {
        printf("ERROR: Failed to write correct number of items to output file %s: %zd of %d\n", outputFile, numElementsWritten, 1);
        free(outputYcbcrData);
        stbi_image_free(rgbData);
        return -1;
    }
    if (fflush(ycbcrFile) != 0)
    {
        printf("ERROR: Failed to flush output file: %s\n", outputFile);
        free(outputYcbcrData);
        stbi_image_free(rgbData);
        return -1;
    }
    if (fclose(ycbcrFile) != 0)
    {
        printf("ERROR: Failed to close output file: %s\n", outputFile);
        free(outputYcbcrData);
        stbi_image_free(rgbData);
        return -1;
    }
    clock_t writeOutEndTime = clock();

#if DEBUG_RGB_to_YCbCr
    const char* ycbcrDebugFileName = "data/debug_ycbcr.bmp";
    if (stbi_write_bmp(ycbcrDebugFileName, imageWidth, imageHeight, 3, rgbData) == 0)
    {
        printf("ERROR: Failed to write out YCbCr debug image using stb\n");
        stbi_image_free(rgbData);
        return -1;
    }
    printf("INFO: Wrote YCbCr debug image to file: %s\n", ycbcrDebugFileName);
#endif

    // Cleanup
    free(outputYcbcrData);
    stbi_image_free(rgbData);

    // Timing calculations
    int imageLoadTime = (int)(((float)(imageLoadEndTime - imageLoadStartTime) / CLOCKS_PER_SEC) * 1000000.0f);
    int conversionTime = (int)(((float)(conversionEndTime - conversionStartTime) / CLOCKS_PER_SEC) * 1000000.0f);
    int compressionTime = (int)(((float)(compressionEndTime - compressionStartTime) / CLOCKS_PER_SEC) * 1000000.0f);
    int writeOutTime = (int)(((float)(writeOutEndTime - writeOutStartTime) / CLOCKS_PER_SEC) * 1000000.0f);
    printf("INFO: Image load time: %ins\n", imageLoadTime);
    printf("INFO: Conversion time: %ins\n", conversionTime);
    printf("INFO: Compression time: %ins\n", compressionTime);
    printf("INFO: Write out time: %ins\n", writeOutTime);
    printf("INFO: Total time: %ins\n", imageLoadTime + conversionTime + compressionTime + writeOutTime);

    return 0;
}

#if RGBtoYCbCr_EXECUTABLE
int main(int argc, char** argv)
{
    static const char* helpString =
    "usage: RGBtoYCbCr [-i <input file>] [-f <YCbCr format>] [-e <encoding>] [-o <output file>] [-d]\n\n"
    "example: RGBtoYCbCr -i image.jpg -f I444 -e BT.601 -o image.ycbcr\n\n"
    "arguments:\n"
    "   -d                [optional]  Append details of output image to file name. This includes image dimensions\n"
    "                                 and YCbCr output format.\n"
    "   -e <encoding>     [required]  ITU-R Recommendation encoding. Supported encodings are BT.601, BT.709 and BT.2020.\n"
    "   -f <YCbCr format> [required]  Format of YCbCr output data. Supported formats are I444, I420, YV12, I422, NV12,\n"
    "                                 NV21, NV16, NV61, NV24, NV42, UYVY (Y422 and UYNV),\n"
    "                                 YUYV (V422, YUNV and YUY2) and YVYU.\n"
    "                                 (see https://wiki.videolan.org/YCbCr for a description of each format)\n"
    "   -i <input file>   [required]  Path to RGB input file. Supported color spaces are R (greyscale),\n"
    "                                 RA, RGB and RGBA for image formats JPEG, PNG, BMP, TGA and PNM.\n"
    "   -o <output file>  [required]  Name of output file.\n";

    // Parameters
    char* inputFile = NULL;
    YCbCrEncoding ycbcrEncoding = 0;
    YCbCrFormat ycbcrFormat = 0;
    char* outputFile = NULL;
    bool appendDetails = false;

    // Parse arguments
    static const char* helpArg = "--help";
    static const char* detailsArg = "-d";
    static const char* ycbcrEncodingArg = "-e";
    static const char* ycbcrFormatArg = "-f";
    static const char* inputFileArg = "-i";
    static const char* outputFileArg = "-o";
    if (argc < 2)
    {
        printf("Insufficient number of arguments; running 'RGBtoYCbCr --help'\n");
        printf("%s", helpString);
        return -1;
    }
    int argIndex = 1;
    while (argIndex < argc)
    {
        // Grab current argument
        const char* currentArg = argv[argIndex];

        // Special case: help argument
        if (strcmp(currentArg, helpArg) == 0)
        {
            printf("%s", helpString);
            return 0;
        }

        // Special case: append details argument
        if (strcmp(currentArg, detailsArg) == 0)
        {
            appendDetails = true;
            argIndex++;
            continue;
        }

        // Get next argument, as each of the remaining 'keys' require a 'value'
        if (argIndex + 1 >= argc)
        {
            printf("ERROR: Insufficient number of arguments; one more is needed after %s\n", currentArg);
            return -1;
        }
        char* nextArg = argv[argIndex + 1];
        // Rest of arguments
        if (strcmp(currentArg, ycbcrEncodingArg) == 0)
        {
            bool foundYCbCrEncoding = false;
            for (int i = BT601; i <= BT2020; i++)
            {
                if (strcmp(YCbCrEncodingNames[i], nextArg) == 0)
                {
                    ycbcrEncoding = (YCbCrEncoding)i;
                    argIndex += 2;
                    foundYCbCrEncoding = true;
                    break;
                }
            }
            if (!foundYCbCrEncoding)
            {
                printf("ERROR: Unsupported YCbCr format %s\n", nextArg);
                return -1;
            }
        }
        else if (strcmp(currentArg, ycbcrFormatArg) == 0)
        {
            bool foundYCbCrFormat = false;
            for (int i = I444; i <= YVYU; i++)
            {
                if (strcmp(YCbCrFormatNames[i], nextArg) == 0)
                {
                    ycbcrFormat = (YCbCrFormat)i;
                    argIndex += 2;
                    foundYCbCrFormat = true;
                    break;
                }
            }
            if (!foundYCbCrFormat)
            {
                printf("ERROR: Unsupported YCbCr format %s\n", nextArg);
                return -1;
            }
        }
        else if (strcmp(currentArg, inputFileArg) == 0)
        {
            inputFile = nextArg;
            argIndex += 2;
        }
        else if (strcmp(currentArg, outputFileArg) == 0)
        {
            outputFile = nextArg;
            argIndex += 2;
        }
        else
        {
            printf("ERROR: Unsupported argument %s\n", currentArg);
            return -1;
        }
        
    }

    // Execute conversion
    if (RGBtoYCbCr(inputFile, ycbcrEncoding, ycbcrFormat, outputFile) == 0)
    {

        printf("INFO: Conversion was successful\n");
        return 0;
    }
    else
    {
        printf("ERROR: Conversion was unsuccessful\n");
        return -1;
    }
}
#endif

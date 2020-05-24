#ifndef RGB_to_YCbCr_H
#define RGB_to_YCbCr_H

typedef enum YCbCrFormat
{
    // Three plane formats
    I444 = 0,
    I420 = 1,
    YV12 = 2,
    I422 = 3,

    // Two plane formats
    NV12 = 4,
    NV21 = 5,
    NV16 = 6,
    NV61 = 7,
    NV24 = 8,
    NV42 = 9,

    // Packed formats
    UYVY = 10, Y422 = 11, UYNV = 12,
    YUYV = 13, V422 = 14, YUNV = 15, YUY2 = 16,
    YVYU = 17

} YCbCrFormat;
const char* YCbCrFormatNames[] =
{
    "I444",
    "I420",
    "YV12",
    "I422",

    "NV12",
    "NV21",
    "NV16",
    "NV61",
    "NV24",
    "NV42",

    "UYVY", "Y422", "UYNV",
    "YUYV", "V422", "YUNV", "YUY2"
    "YVYU"
};

typedef enum YCbCrEncoding
{
    BT601  = 0,
    BT709  = 1,
    BT2020 = 2
} YCbCrEncoding;
const char* YCbCrEncodingNames[] =
{
    "BT.601",
    "BT.709",
    "BT.2020"
};

int RGBtoYCbCr(const char* inputFile, YCbCrEncoding outputEncoding, YCbCrFormat outputFormat, const char* outputFile);

#endif
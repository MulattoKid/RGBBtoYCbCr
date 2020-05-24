# RGB -> YCbCr

## Support
### Conversions Methods
- Scalar: ```#define RGBtoYCbCr_SIMD 0``` in ```RGBtoYCbCr.c:10```
- SSE3: ```#define RGBtoYCbCr_SIMD 1``` in ```RGBtoYCbCr.c:10```

### Formats
- I444 (three-plane, 24bpp)
- I420 and YV12 (three-plane, 12bpp)
- I422 (three-plane, 16bpp)
- NV12 and NV21 (two-plane, 12bpp)
- NV16 and NV61 (two-plane, 16bpp)
- NV24 and NV42 (two-plane, 24bpp)
- UYVY = Y422 = UYNV (packed, 16bpp)
- YUYV = V422 = YUNV = YUY2 (packed, 16bpp)
- YVYU (packed, 16bpp)

### Color Encodings
- BT.601
- BT.709
- BT.2020

## Build
### Windows
```
git clone https://github.com/MulattoKid/RGBtoYCbCr.git
cd RGBtoYCbCr/build
cmake -G "<Visual Studio Version>" ..
cmake --build . --target RGBtoYCbCr --config <DEBUG/RELWITHDEBINFO/RELEASE>
```

### Linux
```
git clone https://github.com/MulattoKid/RGBtoYCbCr.git
cd RGBtoYCbCr/build
cmake -DCMAKE_BUILD_TYPE=<Debug/Release> ..
make RGBtoYCbCr -jN
```

## Run
```
./RGBtoYCbCr -i <input file> -f <YCbCr format> -e <color encoding> -o <output file>

## Incorporate Into Your Own Project
1) Add the respective header and source file to your build system.
2) Note that the files in ```src/external``` directory are also needed.
3) Finally, the <RGBtoYCbCr/YCbCrtoRGB>.c contains to defines that need to be changed from 1 to 0. They are located at the top of the respective file: ```#define <RGBtoYCbCr_EXECUTABLE/YCbCrtoRGB_EXECUTABLE>```.

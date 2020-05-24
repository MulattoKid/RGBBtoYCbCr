# YCbCr <-> RGB

## Support
### Conversions
- RGB -> YCbCr: YES (scalar and SIMD)
- YCbCr -> RGB: NO

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
git clone https://github.com/MulattoKid/YCbCrtoRGB.git
cd YCbCr
mkdir build
cmake -G "<Visual Studio Version>" ..
cmake --build . --target <RGBtoYCbCr/YCbCrtoRGB> --config <DEBUG/RELWITHDEBINFO/RELEASE>
```

### Linux
```
git clone https://github.com/MulattoKid/YCbCrtoRGB.git
cd YCbCr
mkdir build
cmake -DCMAKE_BUILD_TYPE=<Debug/Release> ..
make <RGBtoYCbCr/YCbCrtoRGB> -jN
```

## Compile Shaders
```
glslangValidator.exe -V -o I444.spv I444.comp
```

## Run
### RGB to YCbCr
```
./RGBtoYCbCr -i <input file> -f <YCbCr format> -e <color encoding> -o <output file>
```

### YCbCr to RGB
```
TODO
```

## Incorporate Into Your Own Project
1) Add the respective header and source file to your build system.
2) Note that the files in ```src/external``` directory are also needed.
3) Finally, the <RGBtoYCbCr/YCbCrtoRGB>.c contains to defines that need to be changed from 1 to 0. They are located at the top of the respective file: ```#define <RGBtoYCbCr_EXECUTABLE/YCbCrtoRGB_EXECUTABLE>```.

## Perf
### Setup Perf
```bash
sudo sh -c 'echo "-1" > /proc/sys/kernel/perf_event_paranoid'
sudo sh -c 'echo "0" > /proc/sys/kernel/kptr_restrict'
```

### Compile Release with Symbols
```bash
make release_perf
```

### Use Perf
```bash
perf record -g -F 3998 -e cycles -e cache-misses ./build/RGBtoYCbCr -i data/lando_norris_1600x900.jpg -f I444 -e BT.601 -o data/output.ycbcr
```

### Get Perf Report
```bash
perf report
```

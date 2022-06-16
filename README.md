# realsense-tools

```console
git clone https://github.com/xEnVrE/realsense-tools
cd realense-tools
mkdir build
cd build
cmake ../
make
```

#### Test
```console
./bin/device
yarpview --synch --name /test/rgb:i
yarpview --synch --name /test/depth:i
yarp connect /depthCamera/rgbImage:o /test/rgb:i
yarp connect /depthCamera/depthImage:o /test/depth:i fast_tcp+recv.portmonitor+type.dll+file.depthimage_to_rgb
```

#### Suggestions for `librealsense` build

Make sure you have `CUDA` installed

```console
git clone https://github.com/intelrealsense/librealsense
cd librealsense
mkdir build
cd build
cmake -DBUILD_WITH_OPENMP=OFF -DBUILD_WITH_CUDA=ON -DFORCE_RSUSB_BACKEND ../
make
```

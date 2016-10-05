ORB_SLAM 2
===========================

> ORB_SLAM not tested

Dependencies
-----------

```bash
# prebuilt
sudo apt-get install libblas-dev libusb-dev
sudo apt-get install liblapack-dev

# pangolin
sudo apt-get install libglew-dev
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev
sudo apt-get install libdc1394-22-dev libraw1394-dev
sudo apt-get install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev
```

Installation
-----------

```bash
cd ~/External
git clone https://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install

cd ~/External
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout -f v0.4
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 Duvc_ICLUDE_DIRS=/usr/local/include;/usr/include/libusb-1.0..

make -j3

cd ~/External
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

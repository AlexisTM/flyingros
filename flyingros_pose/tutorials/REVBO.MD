Revbo installation - https://github.com/JuanTarrio/rebvo
===========================

Dependencies
-----------

```
# NE10 - for optimisations
cd ~/Workspace/External
git clone https://github.com/projectNe10/Ne10
mkdir Ne10/build
cd Ne10/build
cmake .. -DNE10_LINUX_TARGET_ARCH=armv7 -DGNULINUX_PLATFORM -DCMAKE_BUILD_TYPE=Release

# TooN
cd ~/Workspace/External
git clone https://github.com/edrosten/TooN
cd TooN
./configure && make && sudo make install
```

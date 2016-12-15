Odroid XU3(lite)/XU4 specific
===============================

Reboot without sudo 
-------------------

```
chmod +s /sbin/halt
chmod +s /sbin/reboot
```

Improve performances with CFLAGS
-------------------

Improve compiled programs performances by adding CFLAGS at the end of your .bashrc. If you don't know what CFLAGS is, don't do it.

```
echo export CFLAGS="-O3 -pipe -march=armv7-a -mfloat-abi=hard -mfpu=vfpv3 -fexpensive-optimizations -fprefetch-loop-arrays" >> ~/.bashrc
echo export ARM_ARCHITECTURE=True >> ~/.bashrc
```

SWAP for more "RAM" to avoid freeze on compilations
-------------------

```
touch /swapfile
sudo chmod 600 /swapfile
sudo fallocate -l 4G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon -s
```

#!/bin/bash
if [ ! -d "ethercat-hg" ]; then
    hg clone http://hg.code.sf.net/p/etherlabmaster/code ethercat-hg
fi
cd ethercat-hg
#hg update stable-1.5
./bootstrap
./configure --disable-8139too
make all modules
sudo make modules_install install
sudo depmod
sudo ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo cp /opt/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
#vi /etc/sysconfig/ethercat
echo
echo "--- Please edit /etc/sysconfig/ethercat to your needs! ---"

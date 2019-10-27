#!/bin/sh
./uninstall.sh
sudo dkms install .
sudo modprobe hid-sony
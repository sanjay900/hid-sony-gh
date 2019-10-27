#!/bin/sh
sudo modprobe -r hid-sony
sudo dkms remove hid-sony/0.1 --all || true
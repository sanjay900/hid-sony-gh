#!/bin/sh
sudo modprobe -r hid-guitar-hero
sudo dkms remove hid-guitar-hero/0.1 --all || true
#!/bin/sh

sudo install -Dm644 99-dorna-udev.rules /etc/udev/rules.d/99-dorna-udev.rules

sudo udevadm control --reload-rules
sudo udevadm trigger

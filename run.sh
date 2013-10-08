#!/bin/bash

v4l2-ctl -d /dev/video1 -c focus_auto=0 
v4l2-ctl -d /dev/video1 -c focus_absolute=0

./pinball

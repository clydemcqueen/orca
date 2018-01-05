#!/usr/bin/env bash

# Start orca video stream

# To install:           sudo cp orca_video.service /lib/systemd/system
# To start:             sudo systemctl start orca_video.service
# To stop:              sudo systemctl stop orca_video.service
# To start on boot:     sudo systemctl enable orca_video.service
# To not start on boot: sudo systemctl disable orca_video.service

# For USB cam:
#gst-launch-1.0 -v v4l2src device=/dev/video1 do-timestamp=true ! queue ! "video/x-h264, stream-format=(string)byte-stream, alignment=(string)au, width=(int)1920, height=(int)1080, pixel-aspect-ratio=(fraction)1/1, framerate=(fraction)30/1" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.86.105 port=5600

# For Raspicam:
raspivid --nopreview --mode 5 --bitrate 15000000 --intra 1 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=5600

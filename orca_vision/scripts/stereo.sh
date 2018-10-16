#!/usr/bin/env bash

# On each camera Raspberry Pi:
# -- copy stereo.sh to /home/pi
# -- copy stereo.service to /lib/systemd/system
# -- run systemctl enable stereo.service
# -- reboot

# Use port 9001 for left camera and 9002 for right camera

export LD_LIBRARY_PATH=/usr/local/lib/
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games

# 1080P (1440x1080), 30 fps, 15Mbps
#raspivid -n -w 1440 -h 1080 -b 15000000 -fps 30 -t 0 -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=9002

# 640x480, 40 fps, 2Mbps
raspivid -n -w 640 -h 480 -b 2000000 -fps 40 -t 0 -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=9002

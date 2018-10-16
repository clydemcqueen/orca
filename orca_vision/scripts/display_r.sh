#!/usr/bin/env bash

# Display right channel using gst

gst-launch-1.0 -ev udpsrc uri=udp://192.168.7.1:9002 ! queue ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink

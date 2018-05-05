#!/usr/bin/env bash

# Record left and right mp4 files for later processing
# TODO somewhat flaky

gst-launch-1.0 -ev udpsrc port=9001 ! tee name=LEFTOUT ! tee name=LEFTWINDOW ! queue ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! mp4mux ! filesink location=/home/clyde/left.mp4 LEFTOUT. ! queue ! udpsink port=5701 LEFTWINDOW. ! queue ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink udpsrc port=9002 ! tee name=RIGHTOUT ! tee name=RIGHTWINDOW ! queue ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! mp4mux ! filesink location=/home/clyde/right.mp4 RIGHTOUT. ! queue ! udpsink port=5702 RIGHTWINDOW. ! queue ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink

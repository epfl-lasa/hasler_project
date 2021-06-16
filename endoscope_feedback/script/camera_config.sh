#!/bin/bash

/usr/bin/v4l2-ctl -d /dev/video0 -c contrast=20
/usr/bin/v4l2-ctl -d /dev/video0 -c white_balance_temperature=2800
/usr/bin/v4l2-ctl -d /dev/video0 -c sharpness=7
/usr/bin/v4l2-ctl -d /dev/video0 -c exposure_auto=1
/usr/bin/v4l2-ctl -d /dev/video0 -c exposure_absolute=300

#!/bin/bash

set -e
Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0
nohup jupyter notebook --ip 0.0.0.0 --port 8888 --no-browser --allow-root > out.log &

exec "$@"
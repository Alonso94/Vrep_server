#!/bin/bash
set -e

git clone https://github.com/Alonso94/Vrep_server
Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0
jupyter notebook --ip 0.0.0.0 --port 8800 --no-browser --allow-root &

exec "$A"
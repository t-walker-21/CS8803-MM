#!/bin/bash
docker run -d --name hackathon_indigo -it \
  --env="DISPLAY" \
  --workdir="/root/catkin_ws" \
  --volume="/home/$USER/CS8803-MM" \
  --volume="/etc/group:/etc/group:ro" \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env QT_X11_NO_MITSHM=1 \
  --network="host" \
  mandyxie/hackathon-indigo:latest

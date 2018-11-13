#!/bin/bash
docker run -d --name CS8803-MM-indigo -it \
  --env="DISPLAY" \
  --workdir="/root" \
  --volume="/home/$USER/CS8803-MM:/root/CS8803-MM" \
  --volume="/etc/group:/etc/group:ro" \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env QT_X11_NO_MITSHM=1 \
  --network="host" \
  mandyxie/cs8803-mm:latest

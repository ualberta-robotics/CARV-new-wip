set -e

# First, ensure the host allows the connection
xhost +local:$(whoami)

# ensure fresh container
docker rm -f quest3_receiver 2>/dev/null || true
docker build -t quest3_receiver:latest .

# Run the container with authority mounting
docker run -it --rm \
  --name quest3_receiver \
  --privileged \
  --network host \
  --ipc=host \
  --security-opt label=disable \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v $XAUTHORITY:/tmp/.Xauthority:Z \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/workspace:/workspace:Z \
  -v /dev:/dev \
  --device /dev/dri:/dev/dri \
  quest3_receiver:latest \
  /bin/bash -c "cd /workspace && ./run.sh"
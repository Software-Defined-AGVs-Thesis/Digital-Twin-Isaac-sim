#!/bin/bash
xhost +
docker run --name isaac-sim-omnilrs-container -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --ipc=host \
-v $HOME/.Xauthority:/root/.Xauthority \
-e DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-e "PRIVACY_CONSENT=Y" \
-v ${PWD}:/workspace/omnilrs \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v bash_command_history:/commandhistory \
-e XR_RUNTIME_JSON=/openxr_host/active_runtime.json \
-e XDG_RUNTIME_DIR=/run/user/1000 \
-v /home/g04-f25/.config/openxr/1:/openxr_host:ro \
-v /home/g04-f25/.steam/debian-installation:/steam:ro \
-v /run/user/1000:/run/user/1000:rw \
isaac-sim-omnilrs:latest

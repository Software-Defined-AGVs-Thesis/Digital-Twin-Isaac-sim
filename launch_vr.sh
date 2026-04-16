#!/bin/bash
cd ~/OmniLRS
xhost +local:docker
sudo docker run -it --rm --gpus all --privileged --network=host -e ACCEPT_EULA=Y -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json -v /usr/share/vulkan/icd.d/nvidia_icd.json:/usr/share/vulkan/icd.d/nvidia_icd.json:ro -v /tmp/.X11-unix:/tmp/.X11-unix -v /run/user/1000:/run/user/1000 -v $(pwd):/workspace/omnilrs -v /home/g04-f25/.steam/debian-installation/steamapps/common/SteamVR:/steam/steamapps/common/SteamVR -v /home/g04-f25/.steam/debian-installation/steamapps/common/SteamVR/steamxr_linux64.json:/home/g04-f25/.steam/debian-installation/steamapps/common/SteamVR/steamxr_linux64.json:ro -v /home/g04-f25/.steam/debian-installation:/home/g04-f25/.steam/debian-installation:ro \
  -v /home/g04-f25/.config/openxr:/openxr_host isaac-sim-omnilrs:latest /isaac-sim/python.sh /workspace/omnilrs/run.py

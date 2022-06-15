# Overview

This is a ROS node that takes images and detects whether a T or CT player model from CS:GO is present. We prioritize detection on GPU (supports both both nVidia and AMD) and falls back on CPU if no CUDA devices are found.

## Config
 - `screen_msg` is the image topic we subscribe to for detections.
 - `viz` is a flag dictating whether we want to visualize our detections via publishing an annotated image.
 - `weights_dir` is the path to the directory where we load our NN weights.
 - `rate` doesn't do anything for now, we publish immediately in the callback function.

TODO:
 - Set up publishing at a set rate.

# Running

After you clone the repo, make sure to clone the submodule as well. Documentation [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules)

After that you have two options:
1. Run locally (for nVidia and CPU)
2. Run via docker (for AMD)

## Local

Add include/sequoia to python path:

```
cd player_detection
export PYTHONPATH="$PYTHONPATH:$(pwd)/include/sequoia/"
```

You can probably find a better way to add the package to our dependencies but I couldn't figure it out so I'm just lazily adding the repo to PATH.

Now we can launch our node. Remember to `source devel/setup.bash` first.

`roslaunch player_detection detect.launch weights_dir:=$(rospack find player_detection)/include/sequoia/`

## Docker

The node will by default look for a CUDA device and will fall back to CPU if not present. The dockerfile extends ROCm pytorch image so AMD devices are supported as well. For a full list of supported devices, visit the [ROCm docs](https://docs.amd.com/bundle/ROCm-Getting-Started-Guide-v5.1.3/page/Overview_of_ROCm_Installation.html#d1353e146)

To build dockerfile:

`sudo docker build -f ./Dockerfile -t csgo_bot/player_detection .`

To run the dockerfile:
```
sudo docker run -it --cap-add=SYS_PTRACE --security-opt seccomp=unconfined --device=/dev/kfd --device=/dev/dri --group-add video --ipc=host --shm-size 8G \
--mount type=bind,source="$HOME"/ws/csgo_bot/src/player_detection/,target=/ws/src/player_detection \
--network host \
csgo_bot/player_detection:latest

```

Remember to `source /ws/setup.sh` upon entering. Then we can launch the node from the container without needing to specify the weights directory.

`roslaunch player_detection detect.launch`

# Performance

I'm using a Ryzen 7 5900X with 64Gb DDR4-3600 cl 18 and a RX 6800XT. I'm feeding image data published from my [screencap node](https://github.com/raytsou/screencap), which caps at 84 fps. 

According to `rostopic hz`, we detect at around 70-75hz for publishing rate when detecting on GPU with visualization off. I'm sure it can go higher if I offload the screencap node to a different device.

With visualization on, the rate drops to around 50-55hz. 

# Acknowledgements

This package would not be possible without the help of [Igor Rocha](https://github.com/IgaoGuru/). The player detection node relies on his project [Sequoia](https://github.com/IgaoGuru/Sequoia), which is a CS:GO player detector.
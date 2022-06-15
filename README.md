# player_detection

```
sudo docker run -it --cap-add=SYS_PTRACE --security-opt seccomp=unconfined --device=/dev/kfd --device=/dev/dri --group-add video --ipc=host --shm-size 8G \
--mount type=bind,source="$HOME"/ws/csgo_bot/src/player_detection/,target=/app \
-w /app \
--network host \
csgo_bot/player_detection:latest

```
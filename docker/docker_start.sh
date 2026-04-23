sudo docker run -it -d \
  --privileged \
  -v ${HOME}/zzj/geditor:${HOME}/zzj/geditor \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=unix$DISPLAY \
  -e GDK_SCALE \
  -e GDK_DPI_SCALE \
  --name geditor \
  --hostname GLAM \
  --net host \
  hub.zhipeng.zone/hd_map/geditor/ubuntu22.04:app bash

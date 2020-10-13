# 需要安装的东西

## 安装opencv

参考钉钉

## 其他

```bash
sudo apt-get install ros-dashing-async-web-server-cpp
sudo apt-get install ros-dashing-web-video-server
sudo apt install -y libavcodec-dev libavformat-dev libavdevice-dev libavfilter-dev libavutil-dev libswscale-dev
```

## 定义topic

注意如果是compressed后缀加/compressed

例子topic /image_raw/compressed

http://localhost:8080/stream?topic=/image_raw&type=ros_compressed 来显示



## 官方文档

http://wiki.ros.org/web_video_server
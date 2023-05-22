# SBEL Docker noVNC
SBEL Docker Containers with Ubuntu 22.04 + NVIDIA CUDA + VNC + noVNC + Project Chrono

## Instructions
**Pull + Run**
```
docker pull uwsbel/projectchrono_novnc
docker run -d -p 5901:5901 -p 6901:6901 --gpus all uwsbel/projectchrono_novnc
```
Then, navigate to your modern browser and type ```localhost:6901```

**Password**: sbel

**Start, Stop, Remove**
```
docker start <container-name>
docker stop <container-name>
docker rm <container-name>
```
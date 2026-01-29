# FAST_LIO_SAM Docker 使用说明

## 构建Docker镜像

### 方式1：使用构建脚本（推荐）

从项目根目录执行：

```bash
cd /home/fg/Codes/FAST_LIO_SAM
./FAST_LIO_SAM/docker/build.sh
```

### 方式2：手动构建

从项目根目录（`/home/fg/Codes/FAST_LIO_SAM`）执行以下命令：

```bash
docker build -f FAST_LIO_SAM/docker/Dockerfile -t fast_lio_sam:noetic .
```

## 运行Docker容器

### 方式1：使用启动脚本（推荐）

使用提供的启动脚本，自动配置GUI、工作空间映射和/media路径映射：

```bash
cd /home/fg/Codes/FAST_LIO_SAM
./FAST_LIO_SAM/docker/run.sh
```

**启动脚本选项：**

```bash
# 使用默认配置（GUI + 工作空间映射 + /media映射）
./FAST_LIO_SAM/docker/run.sh

# 禁用GUI
./FAST_LIO_SAM/docker/run.sh --no-gui

# 禁用工作空间映射
./FAST_LIO_SAM/docker/run.sh --no-workspace

# 禁用/media映射
./FAST_LIO_SAM/docker/run.sh --no-media

# 自定义容器名称
./FAST_LIO_SAM/docker/run.sh --name my_container

# 查看帮助
./FAST_LIO_SAM/docker/run.sh --help
```

**启动脚本功能：**
- ✅ 自动启用GUI支持（X11转发）
- ✅ 自动映射工作空间到容器内
- ✅ 自动映射/media路径（只读）
- ✅ 自动配置设备访问权限（用于USB设备）
- ✅ 自动配置网络（host模式，用于ROS通信）
- ✅ 智能处理已存在的容器

### 方式2：手动运行

#### 基本运行（无GUI）

```bash
docker run -it --rm \
    --name fast_lio_sam \
    fast_lio_sam:noetic
```

#### 运行并启用GUI（使用X11转发）

```bash
xhost +local:docker
docker run -it --rm \
    --name fast_lio_sam \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /home/fg/Codes/FAST_LIO_SAM/FAST_LIO_SAM:/root/catkin_ws/src/FAST_LIO_SAM \
    -v /media:/media:ro \
    --privileged \
    -v /dev:/dev \
    --network host \
    fast_lio_sam:noetic
xhost -local:docker
```

## 在容器内使用

进入容器后，环境已经自动配置好。可以直接运行：

```bash
# 启动Livox激光雷达驱动（如果使用Livox设备）
roslaunch livox_ros_driver livox_lidar_msg.launch

# 启动mapping节点（示例）
roslaunch fast_lio_sam mapping_velodyne16.launch
# 或使用Livox设备
roslaunch fast_lio_sam mapping_avia.launch

# 或者播放bag文件
rosbag play /root/data/your_data.bag
```

## 注意事项

1. **livox_ros_driver**: Dockerfile已自动安装livox_sdk和livox_ros_driver，支持Livox激光雷达使用。

2. **网络配置**: 如果需要在容器内使用ROS网络，需要添加 `--network host` 参数：
   ```bash
   docker run -it --rm --network host fast_lio_sam:noetic
   ```

3. **设备访问**: 如果需要访问USB设备（如激光雷达），需要添加设备权限：
   ```bash
   docker run -it --rm --privileged -v /dev:/dev fast_lio_sam:noetic
   ```

## 依赖说明

Dockerfile已包含以下依赖：
- ROS Noetic Desktop Full
- PCL >= 1.8
- Eigen3 >= 3.3.4
- GTSAM（通过ros-noetic-gtsam安装，使用系统Eigen，避免冲突）
- GeographicLib（从源码编译安装，用于GPS坐标转换）
- Livox SDK（从源码编译安装）
- livox_ros_driver（已克隆到工作空间）
- 所有必需的ROS包

## 故障排除

如果编译失败，可以进入容器手动编译：
```bash
docker exec -it fast_lio_sam bash
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```


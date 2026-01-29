#!/bin/bash

# FAST_LIO_SAM Docker容器启动脚本
# 支持GUI、工作空间映射和/media路径映射

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
WORKSPACE_PATH="$PROJECT_ROOT/FAST_LIO_SAM"

# 默认配置
IMAGE_NAME="fast_lio_sam:noetic"
CONTAINER_NAME="fast_lio_sam"
ENABLE_GUI=true
MAP_WORKSPACE=true
MAP_MEDIA=true

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-gui)
            ENABLE_GUI=false
            shift
            ;;
        --no-workspace)
            MAP_WORKSPACE=false
            shift
            ;;
        --no-media)
            MAP_MEDIA=false
            shift
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --no-gui           禁用GUI支持（X11转发）"
            echo "  --no-workspace     禁用工作空间映射"
            echo "  --no-media         禁用/media路径映射"
            echo "  --name NAME        指定容器名称（默认: fast_lio_sam）"
            echo "  --image IMAGE      指定镜像名称（默认: fast_lio_sam:noetic）"
            echo "  -h, --help         显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                          # 使用默认配置启动"
            echo "  $0 --no-gui                 # 不使用GUI启动"
            echo "  $0 --name my_container      # 使用自定义容器名"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 -h 或 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 检查镜像是否存在
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "错误: 镜像 '$IMAGE_NAME' 不存在"
    echo "请先构建镜像: $SCRIPT_DIR/build.sh"
    exit 1
fi

# 检查容器是否已存在
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "警告: 容器 '$CONTAINER_NAME' 已存在"
    read -p "是否删除现有容器并创建新的? (y/N): " -n 1 -r
    echo
    # if [[ $REPLY =~ ^[Yy]$ ]]; then
    #     echo "删除现有容器..."
    #     docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1
    # else
        echo "启动现有容器..."
        docker start "$CONTAINER_NAME" >/dev/null 2>&1
        docker exec -it "$CONTAINER_NAME" /bin/bash
        exit 0
    # fi
fi

# 构建docker run命令
DOCKER_RUN_CMD="docker run -it --rm"

# 添加容器名称
DOCKER_RUN_CMD="$DOCKER_RUN_CMD --name $CONTAINER_NAME"

# 添加GUI支持（X11转发）
if [ "$ENABLE_GUI" = true ]; then
    if [ -z "$DISPLAY" ]; then
        echo "警告: DISPLAY环境变量未设置，GUI功能可能无法使用"
    else
        xhost +local:docker >/dev/null 2>&1
        DOCKER_RUN_CMD="$DOCKER_RUN_CMD -e DISPLAY=\$DISPLAY"
        DOCKER_RUN_CMD="$DOCKER_RUN_CMD -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
        echo "✓ 已启用GUI支持"
    fi
fi

# 添加工作空间映射
if [ "$MAP_WORKSPACE" = true ]; then
    if [ -d "$WORKSPACE_PATH" ]; then
        DOCKER_RUN_CMD="$DOCKER_RUN_CMD -v $WORKSPACE_PATH:/root/catkin_ws/src/FAST_LIO_SAM"
        echo "✓ 已映射工作空间: $WORKSPACE_PATH -> /root/catkin_ws/src/FAST_LIO_SAM"
    else
        echo "警告: 工作空间路径不存在: $WORKSPACE_PATH"
    fi
fi

# 添加/media路径映射
if [ "$MAP_MEDIA" = true ]; then
    if [ -d "/media" ]; then
        DOCKER_RUN_CMD="$DOCKER_RUN_CMD -v /media:/media:ro"
        echo "✓ 已映射/media路径: /media -> /media (只读)"
    else
        echo "警告: /media路径不存在"
    fi
fi

# 添加设备访问权限（用于USB设备，如激光雷达）
DOCKER_RUN_CMD="$DOCKER_RUN_CMD --privileged"
DOCKER_RUN_CMD="$DOCKER_RUN_CMD -v /dev:/dev"

# 添加网络配置（用于ROS通信）
DOCKER_RUN_CMD="$DOCKER_RUN_CMD --network host"

# 添加镜像名称
DOCKER_RUN_CMD="$DOCKER_RUN_CMD $IMAGE_NAME"

# 显示启动信息
echo ""
echo "=========================================="
echo "启动Docker容器: $CONTAINER_NAME"
echo "镜像: $IMAGE_NAME"
echo "=========================================="
echo ""

# 执行docker run命令
eval $DOCKER_RUN_CMD

# 清理X11权限（如果启用了GUI）
if [ "$ENABLE_GUI" = true ] && [ -n "$DISPLAY" ]; then
    xhost -local:docker >/dev/null 2>&1
fi





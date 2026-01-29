#!/bin/bash

# FAST_LIO_SAM Docker镜像构建脚本

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"

echo "项目根目录: $PROJECT_ROOT"
echo "Dockerfile路径: $SCRIPT_DIR/Dockerfile"

# 检查Dockerfile是否存在
if [ ! -f "$SCRIPT_DIR/Dockerfile" ]; then
    echo "错误: 找不到Dockerfile"
    exit 1
fi

# 构建镜像
echo "开始构建Docker镜像..."
cd "$PROJECT_ROOT"
docker build -f FAST_LIO_SAM/docker/Dockerfile -t fast_lio_sam:noetic .

if [ $? -eq 0 ]; then
    echo "构建成功！"
    echo "可以使用以下命令运行容器:"
    echo "  docker run -it --rm fast_lio_sam:noetic"
else
    echo "构建失败！"
    exit 1
fi





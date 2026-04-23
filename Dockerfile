# 使用官方Ubuntu基础镜像
#FROM ubuntu:22.04
FROM hub.zhipeng.zone/hd_map/geditor/ubuntu:22.04
# 设置时区（可选）
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
 
# 更新软件包列表并安装基础软件包
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    nano \
    build-essential \
    python3 \
    python3-pip \
    libgoogle-glog-dev \
    libglew-dev \
    # libpcl-dev \
    libprotobuf-dev\
    # 添加更多需要的软件包
    && rm -rf /var/lib/apt/lists/*
 
# 设置工作目录
WORKDIR /app

RUN mkdir -p /app/bin
# 复制项目文件到容器中（如果有的话）
COPY ./bin/vdb2pb /app/bin/
COPY ./bin/vdb2dynamic /app/bin/
COPY ./run.sh /app/
 
# 安装Python项目的依赖（如果有的话）
# RUN pip3 install --no-cache-dir protobuf==3.12.4
 
# 暴露端口（如果有的话）
# EXPOSE 8000
 
# 定义容器启动时执行的命令
#CMD ["echo", "Your Docker Ubuntu container is running!"]
 ENTRYPOINT ["bash", "/app/run.sh"]

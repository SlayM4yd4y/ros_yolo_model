# Alap Linux rendszer
FROM ubuntu:22.04


ENV DEBIAN_FRONTEND=noninteractive
# Alapvető csomagok telepítése
RUN apt-get update && apt-get install -y \
    usbutils \
    v4l-utils \
    gphoto2 \
    libopencv-dev \
    python3-opencv \
    python3-pip \
    && apt-get clean
# Telepítsük a git-et
RUN apt-get update && apt-get install -y git

# YOLOv5 klónozása
RUN git clone https://github.com/ultralytics/yolov5 /app/yolov5

# YOLOv5 függőségek telepítése
RUN pip3 install -r /app/yolov5/requirements.txt

# YOLO függőségek telepítése
RUN pip3 install --upgrade pip
RUN pip3 install ultralytics torch torchvision numpy matplotlib

# Munkakönyvtár létrehozása
WORKDIR /app

# Később másoljuk be a projekt fájlokat
COPY . /app/ros_yolo_model


# YOLO modell futtatásához alapértelmezett parancs
CMD ["bash"]

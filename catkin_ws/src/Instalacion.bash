#!/bin/bash

# Actualizar los repositorios
sudo apt update
sudo apt upgrade

# Instalar pip para Python 3
sudo apt install python3-pip

# Instalar PyTorch y sus dependencias
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Instalar networkx versión 2.6.3
pip install networkx==2.6.3

# Instalar TensorBoard
pip install tensorboard

# Ejecutar el lanzamiento de turtlebot3_gazebo
#roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_gazebo turtlebot3_cuadrado.launch

# Instrucciones para Inicializar la Simulación

Este README proporciona los pasos necesarios para cargar y ejecutar la simulación utilizando Docker y ROS en un entorno Gazebo.

### Requisitos Previos

- Docker instalado y configurado
- ROS y Gazebo compatibles con la simulación
- Soporte para GPU (opcional, pero recomendado para un mejor rendimiento)

### Pasos para Ejecutar la Simulación

1. **Cargar la imagen Docker**\
   Ejecuta el siguiente comando para cargar la imagen de la simulación:

   ```bash
   docker load -i DockerTFM.tar
   ```

2. **Iniciar el contenedor Docker**\
   Arranca el contenedor con acceso a los puertos necesarios y soporte para GPU:

   ```bash
   docker run -it -p 8888:8888 -p 6006:6006 -v /home/usuario/TFM:/TFM --name TFM --gpus all tfm
   ```

3. **Acceder al contenedor**\
   Una vez iniciado, estarás dentro del contenedor listo para ejecutar los siguientes comandos.

4. **Compilar el workspace de ROS**\
   Navega al directorio del workspace y compila los paquetes:

   ```bash
   cd /catkin_ws
   catkin_make
   ```

5. **Configurar el path de modelos para Gazebo**\
   Exporta el path para que Gazebo reconozca los modelos personalizados:

   ```bash
   export GAZEBO_MODEL_PATH=/TFM/catkin_ws/src/TFM/models:$GAZEBO_MODEL_PATH
   ```

6. **Lanzar la simulación en Gazebo**\
   Utiliza uno de los siguientes comandos para cargar el mapa correspondiente:

   ```bash
   roslaunch gazebo_simulation <nombre_mapa>
   ```

   Mapas disponibles:

   - `turtlebot3_stage_1.launch`
   - `turtlebot3_largo.launch`
   - `turtlebot3_2.launch`

7. **Ejecutar el archivo principal de la simulación**\
   Cambia al directorio correspondiente y ejecuta el script:

   ```bash
   cd /src/tfm/src
   python Main.py
   ```

---

### Instrucciones para el Uso del Robot Real

Este README también describe los pasos necesarios para inicializar y operar el robot en un entorno real.

### Pasos para Ejecutar el Robot Real

1. **Configurar el ROS Master URI**
   Asegúrate de establecer correctamente la variable de entorno `ROS_MASTER_URI` para conectar tu máquina local con el robot.

2. **Lanzar el Sensor LIDAR y los Motores**

   - Para el sensor LIDAR, ejecuta uno de los siguientes comandos en la terminal del robot:

     ```bash
     roslaunch rplidar_ros rplidar_a3.launch
     ```

     Si el comando anterior no funciona, intenta con:

     ```bash
     roslaunch rplidar_ros rplidar_a3_USB1.launch
     ```

   - Para los motores, utiliza:

     ```bash
     roslaunch turtlebot3_bringup minimal.launch
     ```

3. **Ejecutar el Archivo Principal**
   Desde la terminal local, ejecuta el archivo principal `Main.py`:

   ```bash
   python Main.py
   ```

---

### Notas Adicionales

- Asegúrate de que todas las dependencias estén correctamente instaladas.
- Revisa los permisos de Docker si encuentras problemas con la ejecución.

# TFMMovil

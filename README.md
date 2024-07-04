## src
Contiene el código del proyecto para controlar el dron y ejecutar el sistema de detección y seguimiento. Además de obtener los datos sobre las posiciones de los robots capturados por OptiTrack

Para inicializar el espacio de trabajo, en cada terminal que se utilice, se debe ejecutar el siguiente comando:
```bash
catkin build
source devel/setup.bash
```

## src/pytello
Contiene el código relativo al control del dron

Ejecutar con:
```bash
python3 main.py
```

## src/optitrack_arena
Contiene el código relativo a la obtención de datos de OptiTrack

Para crear los nodos que obtendran y publicarán los datos de las posiciones del dron y Turtlebot capturados por OptiTrack, se debe ejecutar el siguiente comando:
```bash
roslaunch optitrack_arena tello_optitrack.launch
```

Si se quiere visualizar las posiciones y caminos de ambos robots, ejecutar en otra terminal:
```bash
rviz
```



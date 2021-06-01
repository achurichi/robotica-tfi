# Constantes

ROBOT_NAME = 'IRB120'          # Nombre del robot el Coppelia
USE_SAVED_TRAJECTORIES = True  # Usa las trayectorias calculadas previamente
JOINTS_NUM = 6                 # Cantidad de articulaciones
CHECKPOINTS_R1_NUM = 2         # Cantidad de Checkpoints para el Robot 1
CHECKPOINTS_R2_NUM = 6         # Cantidad de Checkpoints para el Robot 1
CHECKPOINTS_PIONEER_NUM = 6    # Checkpoints para las cajas del Pioneer
CHECKPOINTS_BOXES_NUM = 2
CHECKPOINTS_SHELF = 6
BOX_COUNT = 3                  # Cantidad de cajas
POINTS_FAST_TRANSLATION = 50   # Movimiento rápido
POINTS_MID_TRANSLATION = 150   # Movimiento moderado
POINTS_SLOW_TRANSLATION = 350  # Movimiento lento

MOTOR_SPEED = 0.3  # Velocidad del motor
LOOP_DELAY = 0.1   # Delay entre sequencias
COLOR_BLACK = 0.15  # Lectura del sensor de infrarojo para el color negro
COLOR_WHITE = 0.7  # Lectura del sensor de infrarojo para el color blanco
COLOR_GREEN_UP = 0.45 # Lectura del sensor de infrarojo para el color verde
COLOR_GREEN_DOWN = 0.42 #Lectura del sensor de infrarojo para el color verde
COLOR_BLUE_UP = 0.37 # Lectura del sensor de infrarojo para el color azul de la llegada
COLOR_BLUE_DOWN = 0.35 # Lectura del sensor de infrarojo para el color azul de la llegada
COLOR_BLUE_MARK_UP = 0.27 # Lectura del sensor de infrarojo para el color azul de la marca
COLOR_BLUE_MARK_DOWN = 0.24 # Lectura del sensor de infrarojo para el color azul de la marca

TARGET_DISTANCE_F = 0.2 #distancia entre el pioneer y la pared en la llegada
TARGET_DISTANCE_F2 = 0.3#distancia entre el pioneer y la pared en la llegada

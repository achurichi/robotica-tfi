import roboticstoolbox as rtb
import sim
import numpy as np
from utils import *
from constants import *


def inverse_kinematics(pos, orient, robot='R1', base_pos=None, base_orient=None):
    """Cálculo de Cinemática Inversa para IRB120"""

    theta = [0, 0, 0, 0, 0, 0]

    # Cálculo de tita 1
    # Se aplican las transformaciones para la posición de la toolC e
    if base_pos and base_orient:
        T0_EE = get_SE3(base_pos, base_orient).inv() * get_SE3(pos, orient)
    elif robot == 'R1':
        T0_EE = T_base_1_inv * get_SE3(pos, orient)
    elif robot == 'R2':
        T0_EE = T_base_2_inv * get_SE3(pos, orient)
    # Se extrae T6_EE de T0_EE para obtener T0_6
    T0_6 = T0_EE * T_tool_inv

    pm_x = T0_6.eval().data[0][0][3]  # Vector pm en el eje x0
    pm_y = T0_6.eval().data[0][1][3]  # Vector pm en el eje y0

    theta[0] = np.arctan2(pm_y, pm_x)
    theta[0] = np.minimum(theta[0], np.deg2rad(165))  # Restricciones de giro
    theta[0] = np.maximum(theta[0], np.deg2rad(-165))  # Restricciones de giro

    # Cálculo de tita 2
    Cx_prima = np.sqrt(pm_x**2 + pm_y**2)
    Cz_prima = T0_6.eval().data[0][2][3] - 0.29
    C = np.sqrt(Cx_prima**2 + Cz_prima**2)
    A = 0.27
    B = 0.31

    alfa = np.arctan2(Cz_prima, Cx_prima)
    beta_arg = -(B**2 - A**2 - C**2)/(2*A*C)
    beta = np.arctan2(np.sqrt(1 - beta_arg**2), beta_arg)

    theta[1] = np.pi/2 - beta - alfa  # Codo arriba
    # Si no existe solución con el codo arriba se calcula con el codo abajo
    if (np.isnan(theta[1])):
        theta[1] = np.pi/2 + beta - alfa  # Codo abajo
    theta[1] = np.minimum(theta[1], np.deg2rad(110))  # Restricciones de giro
    theta[1] = np.maximum(theta[1], np.deg2rad(-110))  # Restricciones de giro

    # Cálculo de tita 3
    E = 0.302
    rho_arg = E/B
    rho = np.arctan2(rho_arg, np.sqrt(1 - rho_arg**2))
    gamma_arg = -(C**2 - A**2 - B**2)/(2*A*B)
    gamma = np.arctan2(np.sqrt(1 - gamma_arg**2), gamma_arg)

    theta[2] = np.pi - rho - gamma
    theta[2] = np.minimum(theta[2], np.deg2rad(70))  # Restricciones de giro
    theta[2] = np.maximum(theta[2], np.deg2rad(-110))  # Restricciones de giro

    # Cálculos tita 4, 5 y 6
    # Con los parámetros de DH y los valores de tita 1, 2 y 3 se calculan T0_1, T1_2 y T2_3
    T0_1 = rtb.ETS.rz(theta[0], 'rad') * rtb.ETS.tz(0.29)
    T1_2 = rtb.ETS.rx(-np.pi/2, 'rad') * rtb.ETS.rz(-np.pi /
                                                    2, 'rad') * rtb.ETS.rz(theta[1], 'rad')
    T2_3 = rtb.ETS.tx(0.27) * rtb.ETS.rz(theta[2], 'rad')
    T0_3 = T0_1 * T1_2 * T2_3

    T3_6 = T0_3.inv() * T0_6  # Se premultiplica T0_6 por T0_3^-1 para obtener T3_6

    # Se extraen los valores L, G, J, M y N de la matriz T3_6
    L = T3_6.eval().data[0][0][2]
    G = T3_6.eval().data[0][1][0]  # | F I L O |
    J = T3_6.eval().data[0][1][1]  # | G J M P |
    M = T3_6.eval().data[0][1][2]  # | H K N Q |
    N = T3_6.eval().data[0][2][2]  # | 0 0 0 1 |

    # Trabajando con la matriz R3_6 se pueden obtener los valores de tita 4, 5 y 6
    #
    #        | c4*c5*c6-s4*s6     -c4*c5*s6-c6*s4    -c4*s5 |   | F I L |
    # R3_6 = |       c6*s5            -s5*s6           c5   | = | G J M |
    #        | -c5*c6*s4+c4*s6    c5*s4*s6+c4*c6      s4*s5 |   | H K N |
    #
    # Donde: cn = coseno del ángulo tita n
    #        sn = seno del ángulo tita n

    theta[3] = np.arctan2(N, -L)
    theta[3] = np.minimum(theta[3], np.deg2rad(160))  # Restricciones de giro
    theta[3] = np.maximum(theta[3], np.deg2rad(-160))  # Restricciones de giro

    theta[4] = np.arctan2(np.sqrt(1 - M**2), M)
    theta[4] = np.minimum(theta[4], np.deg2rad(120))  # Restricciones de giro
    theta[4] = np.maximum(theta[4], np.deg2rad(-120))  # Restricciones de giro

    theta[5] = np.arctan2(-J, G)
    theta[5] = np.minimum(theta[5], np.deg2rad(400))  # Restricciones de giro
    theta[5] = np.maximum(theta[5], np.deg2rad(-400))  # Restricciones de giro

    if (np.isnan(np.sum(theta))):  # Si hay algún valor de tita no definido
        raise Exception("Posición no permitida")
    else:
        return theta

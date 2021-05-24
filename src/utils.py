import roboticstoolbox as rtb
import sim
import numpy as np


def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart("127.0.0.1", port, True, True, 5000, 5)
    if clientID != -1:
        print("Conectado a ", port)
    else:
        print("No se pudo conectar al servidor")
    return clientID


def get_SE3(pos, orient):
    """Cálculo de la Transformada Homogenea con los valores de posición y orientación"""
    SE3 = rtb.ETS.tx(pos[0]) * rtb.ETS.ty(pos[1]) * rtb.ETS.tz(pos[2])
    SE3 = SE3 * rtb.ETS.rx(orient[0], 'rad') * \
        rtb.ETS.ry(orient[1], 'rad') * rtb.ETS.rz(orient[2], 'rad')
    return SE3


T_tool_inv = rtb.ETS.tx(-0.072 - 0.065446) * rtb.ETS.rx(-np.pi,
                                                        'rad') * rtb.ETS.ry(np.pi/2, 'rad')

T_base_1_inv = rtb.ETS.rz(np.pi, 'rad') * rtb.ETS.tx(-1.14) * rtb.ETS.ty(-0.126) * \
    rtb.ETS.tz(-0.2521)

T_base_2_inv = 1

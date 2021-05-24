from utils import *
from kinematics import *


def get_angles_for_trajectory(points, tray_type, num_movements):
    """Cálculo de trayectorias para IRB120"""
    tray_thetas = []
    num_checkpoints = len(points)
    sub_tray_points = int(num_movements/(num_checkpoints - 1))

    for i in range(num_checkpoints - 1):
        if (tray_type[i] == 'jtraj'):
            sub_tray = rtb.jtraj(
                points[i]['angles'], points[i + 1]['angles'], sub_tray_points)
            if (i == 0):
                tray_thetas = sub_tray.y
            else:
                tray_thetas = np.concatenate((tray_thetas, sub_tray.y))

        elif (tray_type[i] == 'ctraj'):
            sub_tray = rtb.ctraj(get_SE3(points[i]['position'], points[i]['orientation']).eval(),
                                 get_SE3(points[i + 1]['position'],
                                         points[i + 1]['orientation']).eval(),
                                 sub_tray_points)
            # Se pasa de las transformaciones homogeneas a ctraj a los titas
            for j in range(sub_tray_points):
                theta = inverse_kinematics(sub_tray[j].t, sub_tray[j].rpy())
                if (not theta):
                    raise Exception(f"Punto no permitido en la trayectoria")
                elif (i == 0 and j == 0):
                    tray_thetas = np.array([theta])
                else:
                    tray_thetas = np.concatenate(
                        (tray_thetas, np.array([theta])))

    return tray_thetas

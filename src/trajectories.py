from utils import *
from kinematics import *
from constants import *


def get_angles_for_trajectory(points, tray_type, num_movements, robot):
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
                theta = inverse_kinematics(
                    sub_tray[j].t, sub_tray[j].rpy(), robot)
                if (not theta):
                    raise Exception(f"Punto no permitido en la trayectoria")
                elif (i == 0 and j == 0):
                    tray_thetas = np.array([theta])
                else:
                    tray_thetas = np.concatenate(
                        (tray_thetas, np.array([theta])))

    return tray_thetas


def calc_trajectories(home_R1,
                      home_R2,
                      checkpoints_move_box,
                      checkpoints_pioneer,
                      checkpoints_boxes,
                      checkpoints_boxes_shelf,
                      checkpoints_store_box):  # AGREGAR checkpoints_shelf
    traj = {}

    # Trayectorias lado de la cinta
    traj['get_large_box'] = get_angles_for_trajectory([home_R1, checkpoints_move_box[1], checkpoints_boxes[1]],
                                                      ['jtraj', 'ctraj'],
                                                      POINTS_FAST_TRANSLATION,
                                                      'R1')
    traj['move_large_box'] = get_angles_for_trajectory([checkpoints_boxes[1], checkpoints_move_box[1], checkpoints_move_box[0]],
                                                       ['ctraj', 'ctraj'],
                                                       POINTS_MID_TRANSLATION,
                                                       'R1')
    traj['store_large_box_pioneer'] = []
    traj['store_large_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[0]],
                                                                     ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['store_large_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[2]],
                                                                     ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['store_large_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[4]],
                                                                     ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['back_home_large_box'] = []
    traj['back_home_large_box'].append(get_angles_for_trajectory([checkpoints_pioneer[0], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))
    traj['back_home_large_box'].append(get_angles_for_trajectory([checkpoints_pioneer[2], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))
    traj['back_home_large_box'].append(get_angles_for_trajectory([checkpoints_pioneer[4], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))

    traj['get_small_box'] = get_angles_for_trajectory([home_R1, checkpoints_move_box[1], checkpoints_boxes[0]],
                                                      ['jtraj', 'ctraj', 'ctraj'],
                                                      POINTS_FAST_TRANSLATION,
                                                      'R1')
    traj['move_small_box'] = get_angles_for_trajectory([checkpoints_boxes[0], checkpoints_move_box[1], checkpoints_move_box[0]],
                                                       ['ctraj', 'ctraj'],
                                                       POINTS_MID_TRANSLATION,
                                                       'R1')
    traj['store_small_box_pioneer'] = []
    traj['store_small_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[1]],
                                                                     ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['store_small_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[3]], ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['store_small_box_pioneer'].append(get_angles_for_trajectory([checkpoints_move_box[0], checkpoints_pioneer[5]],
                                                                     ['ctraj'],
                                                                     POINTS_MID_TRANSLATION,
                                                                     'R1'))
    traj['back_home_small_box'] = []
    traj['back_home_small_box'].append(get_angles_for_trajectory([checkpoints_pioneer[1], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))
    traj['back_home_small_box'].append(get_angles_for_trajectory([checkpoints_pioneer[3], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))
    traj['back_home_small_box'].append(get_angles_for_trajectory([checkpoints_pioneer[5], checkpoints_move_box[0], home_R1],
                                                                 ['jtraj', 'jtraj'],
                                                                 POINTS_FAST_TRANSLATION,
                                                                 'R1'))

    # Trayectorias lado de la estantería
    traj['prueba'] = get_angles_for_trajectory([home_R2, checkpoints_store_box[0], checkpoints_store_box[2], checkpoints_boxes_shelf[4], home_R2],
                                               ['ctraj', 'ctraj', 'ctraj', 'ctraj'],
                                               POINTS_SLOW_TRANSLATION,
                                               'R2')
    return traj

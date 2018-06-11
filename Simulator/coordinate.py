import numpy as np

def DCM_ENU2Body_euler(azimuth, elevation, roll):
    # [rad]
    DCM_0 = [np.cos(azimuth) * np.cos(elevation), np.sin(azimuth) * np.cos(elevation), -np.sin(elevation)]
    DCM_1 = [-np.sin(azimuth) * np.cos(roll) + np.cos(azimuth) * np.sin(elevation) * np.sin(roll), np.cos(azimuth) * np.cos(roll) + np.sin(azimuth) * np.sin(elevation) * np.sin(roll), np.cos(elevation) * np.sin(roll)]
    DCM_2 = [np.sin(azimuth) * np.sin(roll) + np.cos(azimuth) * np.sin(elevation) * np.cos(roll), -np.cos(azimuth) * np.sin(roll) + np.sin(azimuth) * np.sin(elevation) * np.cos(roll), np.cos(elevation) * np.cos(roll)]
    DCM_ENU2Body_euler = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_ENU2Body_euler

def quat_normalize(quat):
    norm = np.linalg.norm(quat)
    quat = quat / norm
    return quat

def DCM_ENU2Body_quat(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    DCM_0 = [q0 * q0 - q1*q1 - q2*q2 + q3*q3, 2.0 * (q0 * q1 + q2 * q3)    , 2.0 * (q0 * q2 - q1 * q3)]
    DCM_1 = [2.0 * (q0 * q1 - q2 * q3)    , q1*q1 - q0*q0 - q2*q2 + q3*q3, 2.0 * (q1 * q2 + q0 * q3)]
    DCM_2 = [2.0 * (q0 * q2 + q1 * q3)    , 2.0 * (q1 * q2 - q0 * q3)    , q2*q2 - q0*q0 - q1*q1 + q3*q3]
    DCM_ENU2Body_quat = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_ENU2Body_quat

def euler2quat(azimuth, elevation, roll=0.0):
    azimuth = np.radians(azimuth)
    elevation = np.radians(elevation)
    roll = np.radians(roll)

    DCM = DCM_ENU2Body_euler(azimuth, elevation, roll)
    q0 = 0.5 * np.sqrt(1.0 + DCM[0,0] - DCM[1,1] - DCM[2,2])
    q1 = 0.5 * np.sqrt(1.0 - DCM[0,0] + DCM[1,1] - DCM[2,2])
    q2 = 0.5 * np.sqrt(1.0 - DCM[0,0] - DCM[1,1] + DCM[2,2])
    q3 = 0.5 * np.sqrt(1.0 + DCM[0,0] + DCM[1,1] + DCM[2,2])

    quat_max_index = np.argmax([q0, q1, q2, q3])
    if quat_max_index == 0:
        q0 = 0.5 * np.sqrt(1.0 + DCM[0, 0] - DCM[1,1] - DCM[2,2])
        q1 = (DCM[0, 1] + DCM[1, 0]) / (4.0 * q0)
        q2 = (DCM[2, 0] + DCM[0, 2]) / (4.0 * q0)
        q3 = (DCM[1, 2] - DCM[2, 1]) / (4.0 * q0)
    elif quat_max_index == 1:
        q1 = 0.5 * np.sqrt(1.0 - DCM[0, 0] + DCM[1,1] - DCM[2,2])
        q0 = (DCM[0, 1] + DCM[1, 0]) / (4.0 * q1)
        q2 = (DCM[1, 2] + DCM[2, 1]) / (4.0 * q1)
        q3 = (DCM[2, 0] - DCM[0, 2]) / (4.0 * q1)
    elif quat_max_index == 2:
        q2 = 0.5 * np.sqrt(1.0 - DCM[0, 0] - DCM[1,1] + DCM[2,2])
        q0 = (DCM[2, 0] + DCM[0, 2]) / (4.0 * q2)
        q1 = (DCM[1, 2] + DCM[2, 1]) / (4.0 * q2)
        q3 = (DCM[0, 1] - DCM[1, 0]) / (4.0 * q2)
    elif quat_max_index == 3:
        q3 = 0.5 * np.sqrt(1.0 + DCM[0, 0] + DCM[1,1] + DCM[2,2])
        q0 = (DCM[1, 2] - DCM[2, 1]) / (4.0 * q3)
        q1 = (DCM[2, 0] - DCM[0, 2]) / (4.0 * q3)
        q2 = (DCM[0, 1] - DCM[1, 0]) / (4.0 * q3)             

    # q0 = np.cos(azimuth2) * np.cos(elevation2) * np.cos(roll2) + np.sin(azimuth2) * np.sin(elevation2) * np.sin(roll2)
    # q1 = np.cos(azimuth2) * np.cos(elevation2) * np.sin(roll2) - np.sin(azimuth2) * np.sin(elevation2) * np.cos(roll2)
    # q2 = np.cos(azimuth2) * np.sin(elevation2) * np.cos(roll2) + np.sin(azimuth2) * np.cos(elevation2) * np.sin(roll2)
    # q3 = np.sin(azimuth2) * np.cos(elevation2) * np.cos(roll2) - np.cos(azimuth2) * np.sin(elevation2) * np.sin(roll2)

    quat = np.array([q0, q1, q2, q3])
    quat = quat_normalize(quat)

    return quat

def quat2euler(DCM_NED2Body):
    DCM = DCM_NED2Body
    azimuth = np.rad2deg(np.arctan2(DCM[0, 1], DCM[0, 0]))
    elevation = np.rad2deg(-np.arcsin(DCM[0, 2]))
    roll = np.rad2deg(np.arctan2(DCM[1, 2], DCM[2, 2]))

    return azimuth, elevation, roll

if __name__ == '__main__':
    pass



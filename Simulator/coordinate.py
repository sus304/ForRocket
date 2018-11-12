import numpy as np

def DCM_WIND2BODY(alpha, beta):
    '''
    Input: alpha[rad], beta [rad]
    '''
    DCM_0 = [np.cos(alpha) * np.cos(beta), np.cos(alpha) * np.sin(beta), -np.sin(alpha)]
    DCM_1 = [-np.sin(beta), np.cos(beta), 0]
    DCM_2 = [np.sin(alpha) * np.cos(beta), np.sin(alpha) * np.sin(beta), np.cos(alpha)]
    DCM_WIND2BODY = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_WIND2BODY

def DCM_NED2BODY_euler(azimuth, elevation, roll):
    '''
    Input: azimuth, elevation, roll [rad]
    '''
    azi = azimuth
    elv = elevation
    rol = roll
    DCM_0 = [np.cos(azi) * np.cos(elv), np.sin(azi) * np.cos(elv), -np.sin(elv)]
    DCM_1 = [-np.sin(azi) * np.cos(rol) + np.cos(azi) * np.sin(elv) * np.sin(rol), np.cos(azi) * np.cos(rol) + np.sin(azi) * np.sin(elv) * np.sin(rol), np.cos(elv) * np.sin(rol)]
    DCM_2 = [np.sin(azi) * np.sin(rol) + np.cos(azi) * np.sin(elv) * np.cos(rol), -np.cos(azi) * np.sin(rol) + np.sin(azi) * np.sin(elv) * np.cos(rol), np.cos(elv) * np.cos(rol)]
    DCM_NED2BODY_euler = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_NED2BODY_euler

def quat_normalize(quat):
    norm = np.linalg.norm(quat)
    quat = quat / norm
    return quat

def DCM_NED2BODY_quat(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    DCM_0 = [q0*q0 - q1*q1 - q2*q2 + q3*q3, 2.0 * (q0 * q1 + q2 * q3)    , 2.0 * (q0 * q2 - q1 * q3)]
    DCM_1 = [2.0 * (q0 * q1 - q2 * q3)    , q1*q1 - q0*q0 - q2*q2 + q3*q3, 2.0 * (q1 * q2 + q0 * q3)]
    DCM_2 = [2.0 * (q0 * q2 + q1 * q3)    , 2.0 * (q1 * q2 - q0 * q3)    , q2*q2 - q0*q0 - q1*q1 + q3*q3]
    DCM_NED2BODY_quat = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_NED2BODY_quat

def euler2quat(azimuth, elevation, roll=0.0):
    '''
    Input: azimuth, elevation, roll [deg]
    '''
    azimuth = np.radians(azimuth)
    elevation = np.radians(elevation)
    roll = np.radians(roll)

    DCM = DCM_NED2BODY_euler(azimuth, elevation, roll)
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

def quat2euler(DCM_NED2BODY):
    DCM = DCM_NED2BODY
    azimuth = np.rad2deg(np.arctan2(DCM[0, 1], DCM[0, 0]))
    elevation = np.rad2deg(-np.arcsin(DCM[0, 2]))
    roll = np.rad2deg(np.arctan2(DCM[1, 2], DCM[2, 2]))

    return azimuth, elevation, roll

def DCM_ECI2ECEF(t_sec):
    omega_e = 7.2921151e-5  # [rad/s] for WGS84
    xi = omega_e * t_sec
    DCM_0 = [np.cos(xi), np.sin(xi), 0]
    DCM_1 = [-np.sin(xi), np.cos(xi), 0]
    DCM_2 = [0, 0, 1]
    DCM_ECI2ECEF = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_ECI2ECEF

def DCM_ECEF2NED(pos0_LLH):
    '''
    Input: [deg], [deg], [m]
    '''
    # from MATLAB
    lat = np.deg2rad(pos0_LLH[0])
    lon = np.deg2rad(pos0_LLH[1])
    DCM_0 = [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)]
    DCM_1 = [-np.sin(lon), np.cos(lon), 0]
    DCM_2 = [-np.cos(lat) * np.cos(lon), -np.cos(lat) * np.sin(lon), -np.sin(lat)]
    DCM_ECEF2NED = np.array([DCM_0, DCM_1, DCM_2])
    return DCM_ECEF2NED

def vel_ECI2ECEF(vel_ECI, DCM_ECI2ECEF, pos_ECI):
    omega_e = 7.2921151e-5  # [rad/s] for WGS84
    DCM_0 = [0, -omega_e, 0]
    DCM_1 = [omega_e, 0, 0]
    DCM_2 = [0, 0, 0]
    omega_ECI2ECEF = np.array([DCM_0, DCM_1, DCM_2])
    vel_ECEF = DCM_ECI2ECEF.dot(vel_ECI) - omega_ECI2ECEF.dot(pos_ECI)
    return vel_ECEF

def vel_ECEF2ECI(vel_ECEF, DCM_ECI2ECEF, pos_ECI):
    omega_e = 7.2921151e-5  # [rad/s] for WGS84
    DCM_0 = [0, -omega_e, 0]
    DCM_1 = [omega_e, 0, 0]
    DCM_2 = [0, 0, 0]
    omega_ECI2ECEF = np.array([DCM_0, DCM_1, DCM_2])
    vel_ECI = DCM_ECI2ECEF.transpose().dot(vel_ECEF) + omega_ECI2ECEF.dot(pos_ECI)
    return vel_ECI




import numpy as np
from scipy.spatial.transform import Rotation as Rot

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html
def quat_to_rpy(quat,seq='xyz'):
    qx = quat.x
    qy = quat.y
    qz = quat.z
    qw = quat.w
    rot = Rot.from_quat([qx,qy,qz,qw])
    rx,ry,rz = rot.as_euler(seq,degrees=True)
    return rx,ry,rz

def rad_angle_mean(rads):
    rads = rads[~np.isnan(rads)]
    if len(rads) > 0:
        x = np.sum(np.cos(rads))
        y = np.sum(np.sin(rads))
        rad = np.arctan2(y,x)
        return rad
    else:
        return np.nan

def deg_angle_mean(degs):
    rads = np.deg2rad(degs)
    rad = rad_angle_mean(rads)
    deg = np.rad2deg(rad)
    return deg

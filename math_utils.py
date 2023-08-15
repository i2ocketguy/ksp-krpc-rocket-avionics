import numpy as np

def r2d(x):
    return x*180/np.pi


def d2r(x):
    return x*np.pi/180


def cosd(x):
    return np.cos(d2r(x))


def acosd(x):
    return r2d(np.arccos(x))


def sind(x):
    return np.sin(d2r(x))


def asind(x):
    return r2d(np.arcsin(x))


def tand(x):
    return np.tan(d2r(x))


def atand(x):
    return r2d(np.arctan(x))


def atan2d(x, y):
    return r2d(np.arctan2(x, y))


def norm(x):
    return np.linalg.norm(x)


def unit(x):
        return x/norm(x)


def cross(x, y):
    return np.cross(x, y)


def dot(x, y):
    return np.vdot(x, y)


def vang(x, y):
    x = unit(x)
    y = unit(y)
    return acosd(np.clip(dot(x, y), -1, 1))
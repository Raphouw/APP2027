"""SE(2) geometry in metres and radians."""
import numpy as np

def angle(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def compose(a, b):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], angle(a[2]+b[2])])


def inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2]])


def relative(a, b):
    return compose(inverse(a), b)


def transform(points, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]



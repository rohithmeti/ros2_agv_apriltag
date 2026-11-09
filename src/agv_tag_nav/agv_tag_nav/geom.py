"""Small 3D/2D pose helpers (numpy only)."""
import math
import numpy as np
from geometry_msgs.msg import TransformStamped


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def tf_to_mat(t: TransformStamped):
    r, q = t.transform.translation, t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = (r.x, r.y, r.z)
    return T


def mat_to_tf(T, parent, child, stamp):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
    q = mat_to_quat(T[:3, :3])
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
    return m


def inv(T):
    R, p = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def mat_from_xyyaw(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = (x, y)
    return T


def xyyaw(T):
    return T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])

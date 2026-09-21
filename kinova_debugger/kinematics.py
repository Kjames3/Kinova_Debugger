"""Forward kinematics for the Kinova Gen3 7-DoF arm.

Joint origins come from ros2_kortex's
kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro. Every joint rotates
about its own z axis. Joint angles use the Kortex convention: degrees, as
reported by BaseCyclic feedback.
"""

import math

import numpy as np

# (xyz, rpy) of each joint origin relative to its parent link.
_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.15643), (math.pi, 0.0, 0.0)),
    ((0.0, 0.005375, -0.12838), (math.pi / 2, 0.0, 0.0)),
    ((0.0, -0.21038, -0.006375), (-math.pi / 2, 0.0, 0.0)),
    ((0.0, 0.006375, -0.21038), (math.pi / 2, 0.0, 0.0)),
    ((0.0, -0.20843, -0.006375), (-math.pi / 2, 0.0, 0.0)),
    ((0.0, 0.00017505, -0.10593), (math.pi / 2, 0.0, 0.0)),
    ((0.0, -0.10593, -0.00017505), (-math.pi / 2, 0.0, 0.0)),
)
# bracelet_link -> end_effector_link
_END_EFFECTOR = ((0.0, 0.0, -0.061525), (math.pi, 0.0, 0.0))

JOINT_COUNT = len(_JOINT_ORIGINS)


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw, radians."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def kortex_euler_matrix(theta_x: float, theta_y: float, theta_z: float) -> np.ndarray:
    """Rotation matrix for a Kortex Pose's theta_x/y/z, in degrees."""
    return rpy_matrix(math.radians(theta_x), math.radians(theta_y), math.radians(theta_z))


_ORIGIN_ROTATIONS = tuple(rpy_matrix(*rpy) for _, rpy in _JOINT_ORIGINS)
_ORIGIN_TRANSLATIONS = tuple(np.array(xyz) for xyz, _ in _JOINT_ORIGINS)
_EE_ROTATION = rpy_matrix(*_END_EFFECTOR[1])
_EE_TRANSLATION = np.array(_END_EFFECTOR[0])


def tool_rotation(joints_deg) -> np.ndarray:
    """Orientation of the end-effector frame in the base frame."""
    rotation = np.eye(3)
    for origin, angle in zip(_ORIGIN_ROTATIONS, joints_deg):
        rotation = rotation @ origin @ rot_z(math.radians(angle))
    return rotation @ _EE_ROTATION


def tool_pose(joints_deg, tool_z: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation) of the tool frame, with the tool transform along end-effector z."""
    rotation = np.eye(3)
    position = np.zeros(3)
    for origin_r, origin_t, angle in zip(_ORIGIN_ROTATIONS, _ORIGIN_TRANSLATIONS, joints_deg):
        position = position + rotation @ origin_t
        rotation = rotation @ origin_r @ rot_z(math.radians(angle))
    position = position + rotation @ _EE_TRANSLATION
    rotation = rotation @ _EE_ROTATION
    return position + rotation @ np.array([0.0, 0.0, tool_z]), rotation


def rotation_from_vector(vector) -> np.ndarray:
    """Rodrigues: axis-angle vector (radians) to rotation matrix."""
    vector = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    kx, ky, kz = vector / angle
    k = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def rotation_to_vector(rotation) -> np.ndarray:
    """Inverse of rotation_from_vector."""
    rotation = np.asarray(rotation, dtype=float)
    cos_angle = max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:
        # Near 180 deg the skew part vanishes; recover the axis from the symmetric part.
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0))
        index = int(np.argmax(axis))
        for other in range(3):
            if other != index and rotation[index, other] + rotation[other, index] < 0:
                axis[other] = -axis[other]
        return axis / np.linalg.norm(axis) * angle
    skew = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    return skew / (2.0 * math.sin(angle)) * angle


def angle_between(u, v) -> float:
    """Angle between two vectors, in degrees."""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    cosine = float(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def wrap_degrees(angle: float) -> float:
    """Map an angle to (-180, 180]."""
    return -((-angle + 180.0) % 360.0 - 180.0)

"""Small, training-free SE(3) utilities used by manipulation planning.

The transform convention is explicit throughout this module:

``T_parent_child`` maps coordinates expressed in ``child`` into ``parent``.
Consequently ``T_world_wrist = T_world_object @ T_object_wrist``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterable, Tuple

import numpy as np


def _tuple(values: Iterable[float], length: int, name: str) -> Tuple[float, ...]:
    array = np.asarray(tuple(values), dtype=float)
    if array.shape != (length,):
        raise ValueError(f"{name} must contain {length} values, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return tuple(float(value) for value in array)


def quaternion_wxyz_to_matrix(quaternion_wxyz: Iterable[float]) -> np.ndarray:
    """Convert a unit ``wxyz`` quaternion to a 3x3 rotation matrix."""

    q = np.asarray(_tuple(quaternion_wxyz, 4, "quaternion_wxyz"), dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("quaternion_wxyz must have non-zero norm")
    w, x, y, z = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_wxyz(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a proper 3x3 rotation matrix to a normalized ``wxyz`` quaternion."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(matrix), 1.0, atol=1e-6
    ):
        raise ValueError("rotation must be orthonormal with determinant +1")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    # q and -q encode the same rotation.  A non-negative scalar component gives
    # serialization a deterministic representative except at exactly pi radians.
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)


def rotation_about_z(angle_rad: float) -> np.ndarray:
    value = float(angle_rad)
    if not np.isfinite(value):
        raise ValueError("angle_rad must be finite")
    cosine, sine = float(np.cos(value)), float(np.sin(value))
    return np.asarray(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def rotation_to_xyz_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    """Decompose ``R = Rx(roll) @ Ry(tilt) @ Rz(yaw)``.

    This order matches the three wrist hinge joints in :mod:`allegro_probe.scene`.
    """

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")
    sin_tilt = float(np.clip(matrix[0, 2], -1.0, 1.0))
    tilt = float(np.arcsin(sin_tilt))
    cos_tilt = float(np.cos(tilt))
    if abs(cos_tilt) > 1e-7:
        roll = float(np.arctan2(-matrix[1, 2], matrix[2, 2]))
        yaw = float(np.arctan2(-matrix[0, 1], matrix[0, 0]))
    else:
        # At the gimbal singularity, choose yaw=0 and retain a deterministic roll.
        yaw = 0.0
        roll = float(np.arctan2(matrix[2, 1], matrix[1, 1]))
    return roll, tilt, yaw


@dataclass(frozen=True)
class RigidTransform:
    """A typed rigid transform using the ``T_parent_child`` convention."""

    parent_frame: str
    child_frame: str
    translation_m: Tuple[float, float, float]
    quaternion_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not str(self.parent_frame) or not str(self.child_frame):
            raise ValueError("parent_frame and child_frame must be non-empty")
        translation = _tuple(self.translation_m, 3, "translation_m")
        quaternion = np.asarray(
            _tuple(self.quaternion_wxyz, 4, "quaternion_wxyz"), dtype=float
        )
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("quaternion_wxyz must have non-zero norm")
        quaternion /= norm
        if quaternion[0] < 0.0:
            quaternion *= -1.0
        object.__setattr__(self, "translation_m", translation)
        object.__setattr__(
            self,
            "quaternion_wxyz",
            tuple(float(value) for value in quaternion),
        )

    @property
    def rotation(self) -> np.ndarray:
        return quaternion_wxyz_to_matrix(self.quaternion_wxyz)

    @property
    def matrix(self) -> np.ndarray:
        value = np.eye(4, dtype=float)
        value[:3, :3] = self.rotation
        value[:3, 3] = np.asarray(self.translation_m, dtype=float)
        return value

    @property
    def axis_z_parent(self) -> np.ndarray:
        return self.rotation[:, 2].copy()

    def compose(self, other: "RigidTransform") -> "RigidTransform":
        if self.child_frame != other.parent_frame:
            raise ValueError(
                "transform frame mismatch: "
                f"{self.parent_frame}<-{self.child_frame} cannot compose with "
                f"{other.parent_frame}<-{other.child_frame}"
            )
        matrix = self.matrix @ other.matrix
        return RigidTransform.from_matrix(
            parent_frame=self.parent_frame,
            child_frame=other.child_frame,
            matrix=matrix,
        )

    def inverse(self) -> "RigidTransform":
        rotation = self.rotation.T
        translation = -rotation @ np.asarray(self.translation_m, dtype=float)
        return RigidTransform(
            parent_frame=self.child_frame,
            child_frame=self.parent_frame,
            translation_m=tuple(float(value) for value in translation),
            quaternion_wxyz=matrix_to_quaternion_wxyz(rotation),
        )

    @classmethod
    def from_matrix(
        cls,
        *,
        parent_frame: str,
        child_frame: str,
        matrix: np.ndarray,
    ) -> "RigidTransform":
        value = np.asarray(matrix, dtype=float)
        if value.shape != (4, 4) or not np.all(np.isfinite(value)):
            raise ValueError("matrix must be a finite 4x4 array")
        if not np.allclose(value[3], (0.0, 0.0, 0.0, 1.0), atol=1e-8):
            raise ValueError("matrix must be a homogeneous rigid transform")
        return cls(
            parent_frame=parent_frame,
            child_frame=child_frame,
            translation_m=tuple(float(item) for item in value[:3, 3]),
            quaternion_wxyz=matrix_to_quaternion_wxyz(value[:3, :3]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def z_symmetry_transform(
    frame: str, angle_rad: float, *, child_frame: str | None = None
) -> RigidTransform:
    """Return a deterministic rotation about a primitive's local z axis."""

    return RigidTransform(
        parent_frame=frame,
        child_frame=child_frame or f"{frame}_symmetry",
        translation_m=(0.0, 0.0, 0.0),
        quaternion_wxyz=matrix_to_quaternion_wxyz(rotation_about_z(angle_rad)),
    )

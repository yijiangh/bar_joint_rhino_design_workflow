"""Shared transform helpers for CAD-backed kinematics and URDF export."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


_EPS = 1e-12


def as_vector(value: Iterable[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Expected a 3D vector.")
    return vector


def unit(vector: Iterable[float], *, tol: float = _EPS) -> np.ndarray:
    vector = as_vector(vector)
    norm = float(np.linalg.norm(vector))
    if norm <= tol:
        raise ValueError("Cannot normalize a near-zero vector.")
    return vector / norm


def orthogonal_to(vector: Iterable[float], *, tol: float = _EPS) -> np.ndarray:
    vector = unit(vector, tol=tol)
    if abs(float(np.dot(vector, np.array([0.0, 0.0, 1.0], dtype=float)))) < 0.95:
        candidate = np.cross(np.array([0.0, 0.0, 1.0], dtype=float), vector)
    else:
        candidate = np.cross(np.array([1.0, 0.0, 0.0], dtype=float), vector)
    return unit(candidate, tol=tol)


def orthonormalize_rotation(rotation: Iterable[Iterable[float]]) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("Expected a 3x3 rotation matrix.")
    u, _, vh = np.linalg.svd(matrix)
    ortho = u @ vh
    if np.linalg.det(ortho) < 0.0:
        u[:, -1] *= -1.0
        ortho = u @ vh
    return ortho


def make_transform(
    *,
    rotation: Iterable[Iterable[float]] | None = None,
    translation: Iterable[float] | None = None,
) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    if rotation is not None:
        transform[:3, :3] = orthonormalize_rotation(rotation)
    if translation is not None:
        transform[:3, 3] = as_vector(translation)
    return transform


def frame_from_axes(
    origin: Iterable[float],
    x_axis: Iterable[float],
    y_axis: Iterable[float],
    z_axis: Iterable[float],
) -> np.ndarray:
    rotation = np.column_stack((unit(x_axis), unit(y_axis), unit(z_axis)))
    return make_transform(rotation=rotation, translation=origin)


def invert_transform(transform: Iterable[Iterable[float]]) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 transform.")
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def transform_point(transform: Iterable[Iterable[float]], point: Iterable[float]) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 transform.")
    homogeneous = np.ones(4, dtype=float)
    homogeneous[:3] = as_vector(point)
    return (matrix @ homogeneous)[:3]


def local_transform(parent_frame: Iterable[Iterable[float]], child_frame: Iterable[Iterable[float]]) -> np.ndarray:
    return invert_transform(parent_frame) @ np.asarray(child_frame, dtype=float)


def rotation_matrix(axis: Iterable[float], angle: float) -> np.ndarray:
    axis = unit(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def rotation_about_local_z(angle: float) -> np.ndarray:
    return make_transform(rotation=rotation_matrix((0.0, 0.0, 1.0), angle))


def translation_transform(offset: Iterable[float]) -> np.ndarray:
    return make_transform(translation=offset)


def align_vectors(source: Iterable[float], target: Iterable[float], *, tol: float = _EPS) -> np.ndarray:
    source_unit = unit(source, tol=tol)
    target_unit = unit(target, tol=tol)
    cross = np.cross(source_unit, target_unit)
    cross_norm = float(np.linalg.norm(cross))
    dot = float(np.clip(np.dot(source_unit, target_unit), -1.0, 1.0))
    if cross_norm <= tol:
        if dot > 0.0:
            return np.eye(3, dtype=float)
        axis = orthogonal_to(source_unit, tol=tol)
        return rotation_matrix(axis, math.pi)
    axis = cross / cross_norm
    angle = math.atan2(cross_norm, dot)
    return rotation_matrix(axis, angle)


def transport_reference_frame(reference_frame: Iterable[Iterable[float]], target_direction: Iterable[float]) -> np.ndarray:
    reference = np.asarray(reference_frame, dtype=float)
    if reference.shape != (4, 4):
        raise ValueError("Expected a 4x4 reference frame.")
    align = align_vectors(reference[:3, 2], target_direction)
    return align @ reference[:3, :3]


def transform_to_xyz_rpy(transform: Iterable[Iterable[float]]) -> tuple[np.ndarray, tuple[float, float, float]]:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 transform.")
    rotation = orthonormalize_rotation(matrix[:3, :3])
    translation = np.asarray(matrix[:3, 3], dtype=float)

    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy <= 1e-9
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return translation, (roll, pitch, yaw)


def project_point_to_line(
    point: Iterable[float],
    line_point: Iterable[float],
    line_direction: Iterable[float],
) -> tuple[np.ndarray, float]:
    point = as_vector(point)
    line_point = as_vector(line_point)
    line_direction = as_vector(line_direction)
    denom = float(np.dot(line_direction, line_direction))
    if denom <= _EPS:
        raise ValueError("Line direction cannot be near zero.")
    t_value = float(np.dot(point - line_point, line_direction) / denom)
    return line_point + t_value * line_direction, t_value


def rotation_preserves_local_z(rotation: Iterable[Iterable[float]], *, tol: float = 1e-6) -> bool:
    matrix = orthonormalize_rotation(rotation)
    z_axis = matrix[:, 2]
    return float(np.linalg.norm(z_axis - np.array([0.0, 0.0, 1.0], dtype=float))) <= tol

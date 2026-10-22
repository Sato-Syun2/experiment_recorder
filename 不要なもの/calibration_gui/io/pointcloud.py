"""PCD/PLY/NumPy point-cloud readers with explicit format handling."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


class _ScalarPreservationError(ValueError):
    """A fallback reader would silently discard intensity/reflectivity."""


@dataclass(frozen=True)
class PointCloud:
    points_m: np.ndarray
    intensity: Optional[np.ndarray] = None
    colors_rgb: Optional[np.ndarray] = None
    source: str = ""

    def __post_init__(self) -> None:
        points = np.asarray(self.points_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Point cloud must be an N x 3 array in meters.")
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        object.__setattr__(self, "points_m", points)
        if self.intensity is not None:
            intensity = np.asarray(self.intensity, dtype=float).reshape(-1)
            if len(intensity) != len(finite):
                raise ValueError("Intensity count must match point count.")
            object.__setattr__(self, "intensity", intensity[finite])
        if self.colors_rgb is not None:
            colors = np.asarray(self.colors_rgb, dtype=float)
            if colors.shape != (len(finite), 3):
                raise ValueError("RGB colors must be an N x 3 array.")
            colors = colors[finite]
            if colors.max(initial=0.0) > 1.0:
                colors = colors / 255.0
            object.__setattr__(self, "colors_rgb", np.clip(colors, 0.0, 1.0))


def _ascii_ply(path: Path) -> PointCloud:
    with path.open("r", encoding="ascii", errors="strict") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY ended before end_header.")
            header.append(line.strip())
            if line.strip() == "end_header":
                break
        if "format ascii 1.0" not in header:
            raise ValueError("PLY is not ASCII; install Open3D to read binary PLY files.")
        try:
            vertex_index = header.index("element vertex")
        except ValueError as exc:
            candidates = [i for i, value in enumerate(header) if value.startswith("element vertex ")]
            if not candidates:
                raise ValueError("PLY has no vertex element.") from exc
            vertex_index = candidates[0]
        vertex_count = int(header[vertex_index].split()[2])
        properties = []
        for line in header[vertex_index + 1 :]:
            if line.startswith("element "):
                break
            if line.startswith("property "):
                parts = line.split()
                if len(parts) >= 3 and parts[1] == "list":
                    raise ValueError("List-valued vertex properties are unsupported in ASCII PLY.")
                if len(parts) >= 3:
                    properties.append(parts[-1])
        rows = []
        for _ in range(vertex_count):
            line = stream.readline()
            if not line:
                raise ValueError("PLY ended before all vertices were read.")
            rows.append([float(value) for value in line.split()])
    table = np.asarray(rows, dtype=float)
    indices = {name: properties.index(name) for name in ("x", "y", "z") if name in properties}
    if len(indices) != 3:
        raise ValueError("PLY vertex properties must include x, y, and z.")
    points = table[:, [indices["x"], indices["y"], indices["z"]]]
    intensity_name = next((name for name in ("intensity", "reflectivity", "scalar_intensity") if name in properties), None)
    intensity = table[:, properties.index(intensity_name)] if intensity_name else None
    rgb_names = ("red", "green", "blue")
    colors = table[:, [properties.index(name) for name in rgb_names]] if all(name in properties for name in rgb_names) else None
    return PointCloud(points, intensity, colors, str(path))


_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def _binary_ply(path: Path) -> PointCloud:
    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY ended before end_header.")
            decoded = line.decode("ascii", errors="strict").strip()
            header.append(decoded)
            if decoded == "end_header":
                break
        format_line = next((line for line in header if line.startswith("format ")), "")
        byte_order = {"binary_little_endian": "<", "binary_big_endian": ">"}.get(format_line.split()[1] if len(format_line.split()) > 1 else "")
        if byte_order is None:
            raise ValueError(f"Unsupported binary PLY format: {format_line}")
        vertex_line = next((line for line in header if line.startswith("element vertex ")), None)
        if vertex_line is None:
            raise ValueError("PLY has no vertex element.")
        count = int(vertex_line.split()[2])
        vertex_pos = header.index(vertex_line)
        properties = []
        for line in header[vertex_pos + 1:]:
            if line.startswith("element "):
                break
            if not line.startswith("property "):
                continue
            fields = line.split()
            if len(fields) != 3 or fields[1] == "list":
                raise ValueError("List-valued vertex properties are unsupported in binary PLY.")
            scalar = _PLY_TYPES.get(fields[1])
            if scalar is None:
                raise ValueError(f"Unsupported PLY vertex scalar type: {fields[1]}")
            properties.append((fields[2], byte_order + scalar))
        names = [name for name, _ in properties]
        if not all(name in names for name in ("x", "y", "z")):
            raise ValueError("PLY vertex properties must include x, y, and z.")
        dtype = np.dtype(properties, align=False)
        data = stream.read(count * dtype.itemsize)
        if len(data) != count * dtype.itemsize:
            raise ValueError("PLY ended before all binary vertices were read.")
    records = np.frombuffer(data, dtype=dtype, count=count)
    points = np.column_stack([records[name] for name in ("x", "y", "z")]).astype(float)
    intensity_name = next((name for name in ("intensity", "reflectivity", "scalar_intensity") if name in names), None)
    intensity = records[intensity_name].astype(float) if intensity_name else None
    rgb_names = ("red", "green", "blue")
    colors = np.column_stack([records[name] for name in rgb_names]) if all(name in names for name in rgb_names) else None
    return PointCloud(points, intensity, colors, str(path))


def _ascii_pcd(path: Path) -> PointCloud:
    with path.open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD ended before DATA header.")
            decoded = line.decode("ascii", errors="strict").strip()
            header_lines.append(decoded)
            if decoded.upper().startswith("DATA "):
                data_type = decoded.split(maxsplit=1)[1].lower()
                break
        if data_type != "ascii":
            raise ValueError(f"PCD DATA {data_type} needs Open3D; it will not be parsed as ASCII.")
        header = {}
        for line in header_lines:
            parts = line.split()
            if parts:
                header[parts[0].upper()] = parts[1:]
        fields = header.get("FIELDS", header.get("FIELD", []))
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        count = int(header.get("POINTS", [str(int(header.get("WIDTH", ["0"])[0]) * int(header.get("HEIGHT", ["1"])[0]))])[0])
        text = stream.read().decode("ascii", errors="strict")
    table = np.loadtxt(text.splitlines(), dtype=float, ndmin=2)
    if len(table) != count:
        raise ValueError(f"PCD header says {count} points, found {len(table)}.")
    if table.shape[1] != sum(counts):
        raise ValueError(f"PCD header describes {sum(counts)} scalar values per point, found {table.shape[1]}.")
    offsets = np.cumsum([0] + counts[:-1]).tolist()
    indices = {name: offsets[fields.index(name)] for name in ("x", "y", "z") if name in fields}
    if len(indices) != 3:
        raise ValueError("PCD fields must include x, y, and z.")
    for name in ("x", "y", "z"):
        if counts[fields.index(name)] != 1:
            raise ValueError(f"PCD {name} field must be scalar.")
    points = table[:, [indices["x"], indices["y"], indices["z"]]]
    intensity_name = next((name for name in ("intensity", "reflectivity") if name in fields), None)
    if intensity_name and counts[fields.index(intensity_name)] != 1:
        raise ValueError(f"PCD intensity field {intensity_name} must be scalar.")
    intensity = table[:, offsets[fields.index(intensity_name)]] if intensity_name else None
    return PointCloud(points, intensity, source=str(path))


def _binary_pcd(path: Path) -> PointCloud:
    with path.open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD ended before DATA header.")
            decoded = line.decode("ascii", errors="strict").strip()
            header_lines.append(decoded)
            if decoded.upper().startswith("DATA "):
                data_type = decoded.split(maxsplit=1)[1].lower()
                break
        header = {}
        for line in header_lines:
            parts = line.split()
            if parts:
                header[parts[0].upper()] = parts[1:]
        fields = header.get("FIELDS", header.get("FIELD", []))
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = [value.upper() for value in header.get("TYPE", [])]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        point_count = int(header.get("POINTS", [str(int(header.get("WIDTH", ["0"])[0]) * int(header.get("HEIGHT", ["1"])[0]))])[0])
        if data_type != "binary":
            if data_type == "binary_compressed" and any(name in fields for name in ("intensity", "reflectivity")):
                raise _ScalarPreservationError(
                    "Compressed PCD contains intensity; Open3D fallback would drop it. "
                    "Use uncompressed PCD or a scalar-preserving reader."
                )
            raise ValueError(f"PCD DATA {data_type} is not supported by the scalar-preserving reader.")
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths do not match.")
        dtype_map = {("F", 4): "f4", ("F", 8): "f8", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8"}
        dtype_fields = []
        for name, size, type_code, count_value in zip(fields, sizes, types, counts):
            scalar = dtype_map.get((type_code, size))
            if scalar is None:
                raise ValueError(f"Unsupported PCD field type {type_code}{size} for {name}.")
            dtype_fields.append((name, "<" + scalar, (count_value,)) if count_value > 1 else (name, "<" + scalar))
        dtype = np.dtype(dtype_fields, align=False)
        data = stream.read(point_count * dtype.itemsize)
        if len(data) != point_count * dtype.itemsize:
            raise ValueError("PCD ended before all binary points were read.")
    records = np.frombuffer(data, dtype=dtype, count=point_count)
    for axis in ("x", "y", "z"):
        if axis not in fields or counts[fields.index(axis)] != 1:
            raise ValueError(f"PCD must have scalar {axis} field.")
    points = np.column_stack([records[name] for name in ("x", "y", "z")]).astype(float)
    intensity_name = next((name for name in ("intensity", "reflectivity") if name in fields), None)
    intensity = None
    if intensity_name:
        if counts[fields.index(intensity_name)] != 1:
            raise ValueError(f"PCD intensity field {intensity_name} must be scalar.")
        intensity = records[intensity_name].astype(float)
    return PointCloud(points, intensity, source=str(path))


def _open3d_read(path: Path) -> PointCloud:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise ValueError(
            f"{path.suffix.upper()} is binary. Install Open3D to read binary PCD/PLY files."
        ) from exc
    cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=True, remove_infinite_points=True)
    if cloud.is_empty():
        raise ValueError(f"No points could be read from {path}.")
    points = np.asarray(cloud.points, dtype=float)
    colors = np.asarray(cloud.colors, dtype=float) if cloud.has_colors() else None
    return PointCloud(points, colors_rgb=colors, source=str(path))


def _header_has_scalar(path: Path, names: set[str]) -> bool:
    """Check scalar names in PCD FIELDS or PLY vertex property declarations."""

    try:
        with path.open("rb") as stream:
            is_ply = path.suffix.lower() == ".ply"
            while True:
                line = stream.readline()
                if not line:
                    return False
                decoded = line.decode("ascii", errors="ignore").strip()
                parts = decoded.split()
                if is_ply and len(parts) >= 3 and parts[0].lower() == "property":
                    if parts[1].lower() != "list" and parts[-1].lower() in names:
                        return True
                if not is_ply and parts and parts[0].upper() in {"FIELDS", "FIELD"}:
                    return bool(names.intersection(item.lower() for item in parts[1:]))
                if decoded.lower() == "end_header" or decoded.upper().startswith("DATA "):
                    return False
    except OSError:
        return False


def load_point_cloud(path: Path) -> PointCloud:
    """Load XYZ in meters. Binary PCD/PLY uses Open3D; ASCII is supported directly."""

    path = path.expanduser().resolve()
    suffix = path.suffix.lower()
    if suffix == ".npy":
        return PointCloud(np.load(path), source=str(path))
    if suffix == ".npz":
        with np.load(path) as archive:
            key = "points" if "points" in archive else "xyz" if "xyz" in archive else None
            if key is None:
                raise ValueError("NPZ must contain a 'points' or 'xyz' N x 3 array.")
            intensity = archive["intensity"] if "intensity" in archive else None
            colors = archive["colors_rgb"] if "colors_rgb" in archive else None
            return PointCloud(archive[key], intensity, colors, str(path))
    if suffix == ".ply":
        try:
            return _ascii_ply(path)
        except (UnicodeDecodeError, ValueError) as exc:
            if "not ASCII" in str(exc) or isinstance(exc, UnicodeDecodeError):
                try:
                    return _binary_ply(path)
                except (UnicodeDecodeError, ValueError) as binary_error:
                    if _header_has_scalar(path, {"intensity", "reflectivity", "scalar_intensity"}):
                        raise ValueError(
                            "Could not read this PLY without risking loss of intensity/reflectivity."
                        ) from binary_error
                    return _open3d_read(path)
            raise
    if suffix == ".pcd":
        try:
            return _ascii_pcd(path)
        except (UnicodeDecodeError, ValueError) as exc:
            if "needs Open3D" in str(exc) or isinstance(exc, UnicodeDecodeError):
                try:
                    return _binary_pcd(path)
                except _ScalarPreservationError:
                    raise
                except (UnicodeDecodeError, ValueError) as binary_error:
                    if _header_has_scalar(path, {"intensity", "reflectivity"}):
                        raise ValueError(
                            "Could not read this PCD layout without risking loss of intensity/reflectivity."
                        ) from binary_error
                    return _open3d_read(path)
            raise
    raise ValueError(f"Unsupported point cloud extension: {suffix or '(none)'}")

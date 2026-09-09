"""Geometry for testing points against Kortex protection zones."""

import math
from dataclasses import dataclass


@dataclass
class Penetration:
    """How far a point sits inside a zone. ``depth`` <= 0 means the point is clear."""

    zone_name: str
    depth: float
    detail: str

    @property
    def violates(self) -> bool:
        return self.depth > 0.0


def _to_zone_frame(point, zone_shape):
    """Express ``point`` (base frame) in the zone's own frame."""
    origin = zone_shape.origin
    dx = point[0] - origin.x
    dy = point[1] - origin.y
    dz = point[2] - origin.z

    o = zone_shape.orientation
    # Kortex sends a 3x3 rotation (zone -> base). Transpose maps base -> zone.
    rows = (
        (o.row1.column1, o.row1.column2, o.row1.column3),
        (o.row2.column1, o.row2.column2, o.row2.column3),
        (o.row3.column1, o.row3.column2, o.row3.column3),
    )
    if not any(any(v for v in row) for row in rows):
        return dx, dy, dz  # unset orientation: treat as identity
    return tuple(sum(rows[r][c] * d for r, d in enumerate((dx, dy, dz))) for c in range(3))


def penetration(point, zone) -> Penetration:
    """Signed penetration depth of ``point`` into ``zone``, in metres.

    Positive means inside. The depth is the smallest distance to any bounding
    face, so it says how far the zone would have to shrink to clear the point.
    """
    from kortex_api.autogen.messages import Base_pb2

    shape = zone.shape
    x, y, z = _to_zone_frame(point, shape)
    dims = list(shape.dimensions)
    kind = shape.shape_type

    if kind == Base_pb2.CYLINDER:
        radius, height = dims[0], dims[1]
        radial = math.hypot(x, y)
        depth = min(radius - radial, height / 2 - abs(z))
        detail = f"cylinder r={radius:.3f} h={height:.3f}; point radial={radial:.3f} z={z:+.3f}"
    elif kind == Base_pb2.SPHERE:
        radius = dims[0]
        dist = math.sqrt(x * x + y * y + z * z)
        depth = radius - dist
        detail = f"sphere r={radius:.3f}; point dist={dist:.3f}"
    elif kind == Base_pb2.RECTANGULAR_PRISM:
        depth = min(dims[0] / 2 - abs(x), dims[1] / 2 - abs(y), dims[2] / 2 - abs(z))
        detail = f"prism {dims[0]:.3f}x{dims[1]:.3f}x{dims[2]:.3f}; point=({x:+.3f},{y:+.3f},{z:+.3f})"
    else:
        return Penetration(zone.name, float("-inf"), f"unsupported shape type {kind}")

    return Penetration(zone.name, depth, detail)

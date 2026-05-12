import json
import sys
import math

TRAVEL_X = 13  # retracted / safe arm position

# ── Geometry helpers ──────────────────────────────────────────────────────────
# All helpers return lists of (y_local, z, x_arm) tuples.


def _ring_yz(cy, cz, x, r, n=16):
    """Circle in the Y-Z plane at a fixed x (arm depth)."""
    return [
        (
            cy + r * math.cos(i * 2 * math.pi / n),
            cz + r * math.sin(i * 2 * math.pi / n),
            x,
        )
        for i in range(n + 1)
    ]


def _ring_xz(y, cz, cx, r, n=16):
    """Circle in the X-Z plane at a fixed y (arm sweeps depth)."""
    return [
        (
            y,
            cz + r * math.sin(i * 2 * math.pi / n),
            cx + r * math.cos(i * 2 * math.pi / n),
        )
        for i in range(n + 1)
    ]


def _ring_xy(cy, z, cx, r, n=16):
    """Circle in the X-Y plane at a fixed z (arm sweeps depth)."""
    return [
        (
            cy + r * math.cos(i * 2 * math.pi / n),
            z,
            cx + r * math.sin(i * 2 * math.pi / n),
        )
        for i in range(n + 1)
    ]


def _helix(cy, cz, x0, x1, r, turns=2, ppt=14):
    """Helical path: winds around the depth (x) axis."""
    total = turns * ppt
    return [
        (
            cy + r * math.cos(i * 2 * math.pi * turns / total),
            cz + r * math.sin(i * 2 * math.pi * turns / total),
            x0 + (x1 - x0) * i / total,
        )
        for i in range(total + 1)
    ]


# ── Shape parameters ──────────────────────────────────────────────────────────
S = 5.0  # box side length
X0 = 14.0  # near (front) arm x
X1 = 19.0  # far  (back)  arm x
CX = (X0 + X1) / 2  # 16.5 — shape x-centre
CY = S / 2  # 2.5  — shape y-centre (local coords)
CZ = 1 + S / 2  # 3.5  — shape z-centre
R = S / 2  # 2.5  — radius for circular shapes

# ── Shape definitions ─────────────────────────────────────────────────────────
# Each shape  → list of strokes
# Each stroke → list of (y_local, z, x_arm) tuples
#
# y_local is offset by `current_y` and optionally mirrored at runtime;
# z is the vertical coordinate; x_arm is the actual arm extension depth.


def _cube():
    return [
        # Bottom face (near)
        [(0, 1, X0), (S, 1, X0), (S, 1 + S, X0), (0, 1 + S, X0), (0, 1, X0)],
        # Top face (far)
        [(0, 1, X1), (S, 1, X1), (S, 1 + S, X1), (0, 1 + S, X1), (0, 1, X1)],
        # Four depth edges
        [(0, 1, X0), (0, 1, X1)],
        [(S, 1, X0), (S, 1, X1)],
        [(S, 1 + S, X0), (S, 1 + S, X1)],
        [(0, 1 + S, X0), (0, 1 + S, X1)],
    ]


def _pyramid():
    apex = (CY, CZ, X1 + 2)  # apex juts out past the far face
    return [
        # Square base (near face)
        [(0, 1, X0), (S, 1, X0), (S, 1 + S, X0), (0, 1 + S, X0), (0, 1, X0)],
        # Four slant edges to apex
        [(0, 1, X0), apex],
        [(S, 1, X0), apex],
        [(S, 1 + S, X0), apex],
        [(0, 1 + S, X0), apex],
    ]


def _prism():
    """Triangular cross-section, depth running in x."""
    return [
        # Front triangle
        [(0, 1, X0), (S, 1, X0), (CY, 1 + S, X0), (0, 1, X0)],
        # Back triangle
        [(0, 1, X1), (S, 1, X1), (CY, 1 + S, X1), (0, 1, X1)],
        # Three connecting edges
        [(0, 1, X0), (0, 1, X1)],
        [(S, 1, X0), (S, 1, X1)],
        [(CY, 1 + S, X0), (CY, 1 + S, X1)],
    ]


def _sphere():
    """Three orthogonal great circles that frame a sphere."""
    return [
        _ring_yz(CY, CZ, CX, R, 24),  # equator       (Y-Z plane)
        _ring_xz(CY, CZ, CX, R, 24),  # meridian      (X-Z plane)
        _ring_xy(CY, CZ, CX, R, 24),  # prime meridian(X-Y plane)
    ]


def _cylinder():
    """Two end-cap circles joined by four evenly-spaced lines."""
    edges = [
        [
            (CY + R * math.cos(a), CZ + R * math.sin(a), X0),
            (CY + R * math.cos(a), CZ + R * math.sin(a), X1),
        ]
        for a in (i * math.pi / 2 for i in range(4))
    ]
    return [
        _ring_yz(CY, CZ, X0, R, 20),  # front cap
        _ring_yz(CY, CZ, X1, R, 20),  # back  cap
        *edges,
    ]


def _helix_shape():
    """One continuous helical stroke winding along the depth axis."""
    return [_helix(CY, CZ, X0, X1, R, turns=2, ppt=14)]


def _diamond():
    """Octahedron: north/south poles + equatorial square."""
    pole_n = (CY, CZ, X1 + 1)  # north (far)
    pole_s = (CY, CZ, X0 - 1)  # south (near)
    eq = [
        (CY + R * math.cos(i * math.pi / 2), CZ + R * math.sin(i * math.pi / 2), CX)
        for i in range(4)
    ]
    strokes = []
    # Four meridians through the equatorial vertices
    for v in eq:
        strokes.append([pole_n, v, pole_s])
    # Close the equatorial ring
    strokes.append([eq[0], eq[1], eq[2], eq[3], eq[0]])
    return strokes


def _torus():
    """Torus approximated by rings stacked around a central circle."""
    R_major = R  # distance from torus centre to tube centre
    R_minor = R * 0.45  # tube radius
    n_rings = 10  # number of tube-cross-section rings
    n_pts = 14  # points per ring
    strokes = []
    for k in range(n_rings + 1):
        phi = k * 2 * math.pi / n_rings
        # Centre of this tube-ring in (y, z, x) space
        rc_y = CY + R_major * math.cos(phi)
        rc_x = CX + R_major * math.sin(phi)  # wraps in the x direction
        ring = [
            (
                rc_y + R_minor * math.cos(t * 2 * math.pi / n_pts),
                CZ + R_minor * math.sin(t * 2 * math.pi / n_pts),
                rc_x,
            )
            for t in range(n_pts + 1)
        ]
        strokes.append(ring)
    return strokes


SHAPES = {
    "CUBE": _cube(),
    "PYRAMID": _pyramid(),
    "PRISM": _prism(),
    "SPHERE": _sphere(),
    "CYLINDER": _cylinder(),
    "HELIX": _helix_shape(),
    "DIAMOND": _diamond(),
    "TORUS": _torus(),
}

# ── Path generation ────────────────────────────────────────────────────────────


def create_shapes_json(shape_sequence, mirror=False):
    points = []
    m_factor = -1 if mirror else 1
    shape_stride = 14

    points.append({"label": "Home", "x": TRAVEL_X, "y": 0, "z": 14})

    total_width = (len(shape_sequence) * shape_stride) - 4
    current_y = -(total_width / 2)

    for shape_name in shape_sequence:
        if shape_name not in SHAPES:
            print(f"  ⚠  Skipping unknown shape: '{shape_name}'")
            continue

        label_tag = f"{'(Mirrored) ' if mirror else ''}─" * 30
        points.append(
            {
                "label": f"── {shape_name} {label_tag}",
                "x": TRAVEL_X,
                "y": current_y * m_factor,
                "z": 14,
            }
        )

        for i, stroke in enumerate(SHAPES[shape_name]):
            if not stroke:
                continue

            # ── Travel to stroke start (arm retracted) ──
            sy = (stroke[0][0] + current_y) * m_factor
            sz = stroke[0][1]
            sx = stroke[0][2]
            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} travel",
                    "x": TRAVEL_X,
                    "y": sy,
                    "z": sz,
                }
            )

            # ── Pen-down: extend to the shape surface ──
            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} pen-down",
                    "x": sx,
                    "y": sy,
                    "z": sz,
                }
            )

            # ── Draw remaining points in the stroke ──
            for pt in stroke[1:]:
                py = (pt[0] + current_y) * m_factor
                pz = pt[1]
                px = pt[2]
                points.append(
                    {"label": f"{shape_name} | draw", "x": px, "y": py, "z": pz}
                )

            # ── Lift (retract arm at stroke end position) ──
            ey = (stroke[-1][0] + current_y) * m_factor
            ez = stroke[-1][1]
            points.append(
                {"label": f"{shape_name} | lift", "x": TRAVEL_X, "y": ey, "z": ez}
            )

        current_y += shape_stride

    points.append({"label": "Home-End", "x": TRAVEL_X, "y": 0, "z": 14})

    scene_name = "-".join(shape_sequence)
    payload = {
        "name": f"Draw {scene_name} {'(Mirrored)' if mirror else ''}",
        "type": "cartesian",
        "speed": 0.7,
        "_notes": [
            f"3D shapes: {', '.join(shape_sequence)}",
            f"X range: {X0}–{X1}  |  Y stride: {shape_stride}",
        ],
        "points": points,
    }

    return json.dumps(payload, indent=2, sort_keys=False), scene_name


# ── Interactive menu ───────────────────────────────────────────────────────────


def main():
    shape_names = ", ".join(sorted(SHAPES.keys()))
    print("\n=== Robot Arm 3D Path Generator ===")
    print(f"Available shapes: {shape_names}")
    print("Example input: CUBE SPHERE HELIX TORUS\n")

    user_input = input("Shapes to draw (space-separated): ").strip().upper()
    if not user_input:
        print("No shapes entered. Exiting.")
        sys.exit()

    shape_list = user_input.split()

    unknown = [s for s in shape_list if s not in SHAPES]
    if unknown:
        print(f"  ⚠  Unknown shape(s) will be skipped: {', '.join(unknown)}")

    mirror_input = input("Mirror the output? (y/n): ").strip().lower()
    is_mirrored = mirror_input == "y"

    json_output, scene_name = create_shapes_json(shape_list, mirror=is_mirrored)

    filename = f"{scene_name.lower()}{'_mirrored' if is_mirrored else ''}.json"
    with open(filename, "w") as f:
        f.write(json_output)

    total_points = json_output.count('"label"')
    print(
        f"\n✅  Saved '{filename}'  ({len(shape_list)} shape(s), {total_points} waypoints)"
    )


if __name__ == "__main__":
    main()

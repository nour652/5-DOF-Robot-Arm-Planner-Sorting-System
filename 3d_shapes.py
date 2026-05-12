"""
robot_arm_3d.py
===============
Generates Cartesian robot-arm waypoint JSON for 3D wireframe shapes.

Point format: (y_local, z, x_arm)
  y_local  – local lateral offset (shifted by current_y at runtime, optionally mirrored)
  z        – vertical coordinate
  x_arm    – arm extension / depth  (TRAVEL_X = 13 when lifted, varies during drawing)

Run:  python robot_arm_3d.py
      python robot_arm_3d.py --demo          (saves a pre-built showcase JSON)
      python robot_arm_3d.py --list          (list all shapes)
"""

import json
import sys
import math

TRAVEL_X = 13

# ── Shared shape parameters ────────────────────────────────────────────────────
S = 5.0
X0 = 14.0
X1 = 19.0
CX = (X0 + X1) / 2  # 16.5
CY = S / 2  # 2.5
CZ = 1 + S / 2  # 3.5
R = S / 2  # 2.5


# ══════════════════════════════════════════════════════════════════════════════
#  Geometry helpers
# ══════════════════════════════════════════════════════════════════════════════


def _ring_yz(cy, cz, x, r, n=16):
    return [
        (
            cy + r * math.cos(i * 2 * math.pi / n),
            cz + r * math.sin(i * 2 * math.pi / n),
            x,
        )
        for i in range(n + 1)
    ]


def _ring_xz(y, cz, cx, r, n=16):
    return [
        (
            y,
            cz + r * math.sin(i * 2 * math.pi / n),
            cx + r * math.cos(i * 2 * math.pi / n),
        )
        for i in range(n + 1)
    ]


def _ring_xy(cy, z, cx, r, n=16):
    return [
        (
            cy + r * math.cos(i * 2 * math.pi / n),
            z,
            cx + r * math.sin(i * 2 * math.pi / n),
        )
        for i in range(n + 1)
    ]


def _helix(cy, cz, x0, x1, r, turns=2, ppt=14):
    total = turns * ppt
    return [
        (
            cy + r * math.cos(i * 2 * math.pi * turns / total),
            cz + r * math.sin(i * 2 * math.pi * turns / total),
            x0 + (x1 - x0) * i / total,
        )
        for i in range(total + 1)
    ]


# ══════════════════════════════════════════════════════════════════════════════
#  Standard shapes
# ══════════════════════════════════════════════════════════════════════════════


def _cube():
    return [
        [(0, 1, X0), (S, 1, X0), (S, 1 + S, X0), (0, 1 + S, X0), (0, 1, X0)],
        [(0, 1, X1), (S, 1, X1), (S, 1 + S, X1), (0, 1 + S, X1), (0, 1, X1)],
        [(0, 1, X0), (0, 1, X1)],
        [(S, 1, X0), (S, 1, X1)],
        [(S, 1 + S, X0), (S, 1 + S, X1)],
        [(0, 1 + S, X0), (0, 1 + S, X1)],
    ]


def _pyramid():
    apex = (CY, CZ, X1 + 2)
    return [
        [(0, 1, X0), (S, 1, X0), (S, 1 + S, X0), (0, 1 + S, X0), (0, 1, X0)],
        [(0, 1, X0), apex],
        [(S, 1, X0), apex],
        [(S, 1 + S, X0), apex],
        [(0, 1 + S, X0), apex],
    ]


def _prism():
    return [
        [(0, 1, X0), (S, 1, X0), (CY, 1 + S, X0), (0, 1, X0)],
        [(0, 1, X1), (S, 1, X1), (CY, 1 + S, X1), (0, 1, X1)],
        [(0, 1, X0), (0, 1, X1)],
        [(S, 1, X0), (S, 1, X1)],
        [(CY, 1 + S, X0), (CY, 1 + S, X1)],
    ]


def _sphere():
    return [
        _ring_yz(CY, CZ, CX, R, 24),
        _ring_xz(CY, CZ, CX, R, 24),
        _ring_xy(CY, CZ, CX, R, 24),
    ]


def _cylinder():
    edges = [
        [
            (CY + R * math.cos(a), CZ + R * math.sin(a), X0),
            (CY + R * math.cos(a), CZ + R * math.sin(a), X1),
        ]
        for a in (i * math.pi / 2 for i in range(4))
    ]
    return [_ring_yz(CY, CZ, X0, R, 20), _ring_yz(CY, CZ, X1, R, 20), *edges]


def _helix_shape():
    return [_helix(CY, CZ, X0, X1, R, turns=2, ppt=14)]


def _diamond():
    pn = (CY, CZ, X1 + 1)
    ps = (CY, CZ, X0 - 1)
    eq = [
        (CY + R * math.cos(i * math.pi / 2), CZ + R * math.sin(i * math.pi / 2), CX)
        for i in range(4)
    ]
    return [[pn, v, ps] for v in eq] + [[eq[0], eq[1], eq[2], eq[3], eq[0]]]


def _torus():
    R_maj, R_min = R, R * 0.45
    n_rings, n_pts = 10, 14
    strokes = []
    for k in range(n_rings + 1):
        phi = k * 2 * math.pi / n_rings
        rc_y = CY + R_maj * math.cos(phi)
        rc_x = CX + R_maj * math.sin(phi)
        strokes.append(
            [
                (
                    rc_y + R_min * math.cos(t * 2 * math.pi / n_pts),
                    CZ + R_min * math.sin(t * 2 * math.pi / n_pts),
                    rc_x,
                )
                for t in range(n_pts + 1)
            ]
        )
    return strokes


# ══════════════════════════════════════════════════════════════════════════════
#  ★ COMPLEX SHOWCASE SHAPES
# ══════════════════════════════════════════════════════════════════════════════


def _dna():
    """
    DNA Double Helix
    Two interleaved helical strands wound along the depth (x) axis,
    joined at regular intervals by base-pair rungs.
    """
    turns, ppt = 3, 18
    total = turns * ppt
    r = R * 1.1

    strand1, strand2 = [], []
    for i in range(total + 1):
        t = i * 2 * math.pi * turns / total
        x = X0 + (X1 - X0) * i / total
        strand1.append((CY + r * math.cos(t), CZ + r * math.sin(t), x))
        strand2.append(
            (CY + r * math.cos(t + math.pi), CZ + r * math.sin(t + math.pi), x)
        )

    strokes = [strand1, strand2]
    for i in range(0, total + 1, ppt // 2):  # base-pair rungs every half-turn
        strokes.append([strand1[i], strand2[i]])
    return strokes


def _trefoil():
    """
    Trefoil Knot
    A single closed curve that loops through 3D space three times before
    closing — topologically impossible to draw in 2D.

    Parametric:  x = sin(t)+2sin(2t),  y = cos(t)-2cos(2t),  z = -sin(3t)
    """
    n = 120
    yz_sc, x_sc = 1.4, 1.8
    pts = []
    for i in range(n + 1):
        t = i * 2 * math.pi / n
        tx = math.sin(t) + 2 * math.sin(2 * t)
        ty = math.cos(t) - 2 * math.cos(2 * t)
        tz = -math.sin(3 * t)
        pts.append((CY + yz_sc * tx, CZ + yz_sc * ty, CX + x_sc * tz))
    return [pts]


def _hyperboloid():
    """
    Hyperboloid of One Sheet (ruled surface)
    Two circles connected by 24 twisted straight ruling lines —
    every single stroke is straight, yet together they sculpt a curved surface.
    """
    n_lines = 24
    twist = n_lines // 3  # 120 degree twist front-to-back
    r = R * 1.15
    strokes = [
        _ring_yz(CY, CZ, X0, r, n_lines),
        _ring_yz(CY, CZ, X1, r, n_lines),
    ]
    for k in range(n_lines):
        a0 = k * 2 * math.pi / n_lines
        a1 = ((k + twist) % n_lines) * 2 * math.pi / n_lines
        strokes.append(
            [
                (CY + r * math.cos(a0), CZ + r * math.sin(a0), X0),
                (CY + r * math.cos(a1), CZ + r * math.sin(a1), X1),
            ]
        )
    return strokes


def _mobius():
    """
    Möbius Strip
    A one-sided surface with a 180° twist.  Drawn as 28 parallel stripes
    swept all the way around; each stripe closes only after circling twice
    because of the half-twist.

    Parametric:
        arm_y  ←  (R + v·cos(u/2))·cos(u)
        arm_z  ←  (R + v·cos(u/2))·sin(u)
        arm_x  ←              v·sin(u/2)
    """
    R_maj = 2.2
    width = 1.8
    n_stripes = 28
    n_pts = 80
    yz_sc, x_sc = 1.1, 1.5

    strokes = []
    for j in range(n_stripes + 1):
        v = -width / 2 + j * width / n_stripes
        stripe = []
        for i in range(n_pts + 1):
            u = i * 2 * math.pi / n_pts
            raw_y = (R_maj + v * math.cos(u / 2)) * math.cos(u)
            raw_z = (R_maj + v * math.cos(u / 2)) * math.sin(u)
            raw_x = v * math.sin(u / 2)
            stripe.append((CY + yz_sc * raw_y, CZ + yz_sc * raw_z, CX + x_sc * raw_x))
        strokes.append(stripe)
    return strokes


def _lissajous_3d():
    """
    3D Lissajous Figure (a=3, b=2, c=5, delta=pi/4)
    Generalises the oscilloscope Lissajous curve into three dimensions.
    """
    a, b, c, delta = 3, 2, 5, math.pi / 4
    n = 200
    pts = []
    for i in range(n + 1):
        t = i * 2 * math.pi / n
        pts.append(
            (
                CY + R * math.sin(a * t + delta),
                CZ + R * math.sin(b * t),
                CX + R * math.sin(c * t),
            )
        )
    return [pts]


# ══════════════════════════════════════════════════════════════════════════════
#  Shape registry
# ══════════════════════════════════════════════════════════════════════════════

SHAPES = {
    "CUBE": _cube(),
    "PYRAMID": _pyramid(),
    "PRISM": _prism(),
    "SPHERE": _sphere(),
    "CYLINDER": _cylinder(),
    "HELIX": _helix_shape(),
    "DIAMOND": _diamond(),
    "TORUS": _torus(),
    # ── Complex ──
    "DNA": _dna(),
    "TREFOIL": _trefoil(),
    "HYPERBOLOID": _hyperboloid(),
    "MOBIUS": _mobius(),
    "LISSAJOUS": _lissajous_3d(),
}

SHOWCASE_SEQUENCE = ["DNA", "TREFOIL", "HYPERBOLOID", "MOBIUS", "LISSAJOUS"]


# ══════════════════════════════════════════════════════════════════════════════
#  Path generation
# ══════════════════════════════════════════════════════════════════════════════


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

        points.append(
            {
                "label": f"── {shape_name} {'(Mirrored)' if mirror else ''}{'─'*40}",
                "x": TRAVEL_X,
                "y": round(current_y * m_factor, 4),
                "z": 14,
            }
        )

        for i, stroke in enumerate(SHAPES[shape_name]):
            if not stroke:
                continue

            sy = round((stroke[0][0] + current_y) * m_factor, 4)
            sz = round(stroke[0][1], 4)
            sx = round(stroke[0][2], 4)

            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} travel",
                    "x": TRAVEL_X,
                    "y": sy,
                    "z": sz,
                }
            )
            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} pen-down",
                    "x": sx,
                    "y": sy,
                    "z": sz,
                }
            )

            for pt in stroke[1:]:
                points.append(
                    {
                        "label": f"{shape_name} | draw",
                        "x": round(pt[2], 4),
                        "y": round((pt[0] + current_y) * m_factor, 4),
                        "z": round(pt[1], 4),
                    }
                )

            points.append(
                {
                    "label": f"{shape_name} | lift",
                    "x": TRAVEL_X,
                    "y": round((stroke[-1][0] + current_y) * m_factor, 4),
                    "z": round(stroke[-1][1], 4),
                }
            )

        current_y += shape_stride

    points.append({"label": "Home-End", "x": TRAVEL_X, "y": 0, "z": 14})

    scene_name = "-".join(shape_sequence)
    payload = {
        "name": f"Draw {scene_name} {'(Mirrored)' if mirror else ''}".strip(),
        "type": "cartesian",
        "speed": 0.7,
        "_notes": [
            f"3D shapes: {', '.join(shape_sequence)}",
            f"Strokes per shape: { {s: len(SHAPES[s]) for s in shape_sequence if s in SHAPES} }",
            f"Total waypoints: {len(points)}",
            f"X depth range: {X0}-{X1}  |  Y stride: {shape_stride}",
        ],
        "points": points,
    }

    return json.dumps(payload, indent=2, sort_keys=False), scene_name


# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════════════


def _print_shape_info():
    rows = [
        ("CUBE", "2 squares + 4 depth edges"),
        ("PYRAMID", "square base + 4 slant edges to apex"),
        ("PRISM", "triangular cross-section, depth in x"),
        ("SPHERE", "3 orthogonal great circles"),
        ("CYLINDER", "2 end-cap rings + 4 connecting lines"),
        ("HELIX", "single helical strand along depth axis"),
        ("DIAMOND", "octahedron: 4 meridians + equatorial ring"),
        ("TORUS", "10 tube-section rings on central circle"),
        None,
        ("DNA", "★  double helix + base-pair rungs (3 turns, 12 rungs)"),
        ("TREFOIL", "★  trefoil knot – 1 closed curve, crosses itself in 3D"),
        ("HYPERBOLOID", "★  ruled surface – 24 straight lines → curved form"),
        ("MOBIUS", "★  Möbius strip – 28 parallel stripes, one-sided surface"),
        ("LISSAJOUS", "★  3D Lissajous figure (a=3, b=2, c=5)"),
    ]
    print("\n┌─ SHAPE LIBRARY ─────────────────────────────────────────────────────┐")
    for row in rows:
        if row is None:
            print("│")
        else:
            print(f"│  {row[0]:<14} {row[1]}")
    print("└─────────────────────────────────────────────────────────────────────┘\n")


def main():
    args = sys.argv[1:]

    if "--list" in args:
        _print_shape_info()
        return

    if "--demo" in args:
        seq = SHOWCASE_SEQUENCE
        print(f"\n⚙  Building showcase: {' → '.join(seq)}")
        json_output, scene_name = create_shapes_json(seq)
        filename = "showcase_demo.json"
        with open(filename, "w") as f:
            f.write(json_output)
        n_pts = json_output.count('"label"')
        print(f"✅  Saved '{filename}'  ({len(seq)} shapes, {n_pts} waypoints)\n")
        return

    # Interactive
    _print_shape_info()
    user_input = input("Shapes to draw (space-separated): ").strip().upper()
    if not user_input:
        print("No shapes entered. Exiting.")
        sys.exit()

    shape_list = user_input.split()
    unknown = [s for s in shape_list if s not in SHAPES]
    if unknown:
        print(f"  ⚠  Unknown shapes skipped: {', '.join(unknown)}")

    mirror_input = input("Mirror the output? (y/n) [n]: ").strip().lower()
    is_mirrored = mirror_input == "y"

    json_output, scene_name = create_shapes_json(shape_list, mirror=is_mirrored)
    filename = f"{scene_name.lower()}{'_mirrored' if is_mirrored else ''}.json"
    with open(filename, "w") as f:
        f.write(json_output)

    n_pts = json_output.count('"label"')
    n_shapes = sum(1 for s in shape_list if s in SHAPES)
    print(f"\n✅  Saved '{filename}'  ({n_shapes} shape(s), {n_pts} waypoints)\n")


if __name__ == "__main__":
    main()

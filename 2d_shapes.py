import json
import sys

# Shape definition: Each shape is a list of "strokes".
SHAPES = {
    "TREE": [
        [(4.5, 1), (5.5, 1), (5.5, 4), (4.5, 4), (4.5, 1)],
        [(2, 4), (8, 4), (5, 7), (2, 4)],
        [(2.5, 6), (7.5, 6), (5, 9), (2.5, 6)],
        [(3, 8), (7, 8), (5, 11), (3, 8)],
    ],
    "CAR": [
        [(2.5, 1), (2, 2), (2.5, 3), (3.5, 3), (4, 2), (3.5, 1), (2.5, 1)],
        [(6.5, 1), (6, 2), (6.5, 3), (7.5, 3), (8, 2), (7.5, 1), (6.5, 1)],
        [(1, 3), (9, 3), (9, 5), (1, 5), (1, 3)],
        [(3, 5), (7, 5), (6, 7.5), (4, 7.5), (3, 5)],
    ],
    "HOUSE": [
        [(2, 1), (8, 1), (8, 6), (2, 6), (2, 1)],
        [(1, 6), (9, 6), (5, 10), (1, 6)],
        [(4, 1), (4, 4), (6, 4), (6, 1)],
    ],
    "STAR": [
        [
            (5, 11),
            (6.5, 6.5),
            (10, 6.5),
            (7, 4.5),
            (8.5, 1),
            (5, 3.5),
            (1.5, 1),
            (3, 4.5),
            (0, 6.5),
            (3.5, 6.5),
            (5, 11),
        ]
    ],
}


def create_shapes_json(shape_sequence, mirror=False):
    points = []
    m_factor = -1 if mirror else 1
    shape_stride = 14

    points.append({"label": "Home", "x": 13, "y": 0, "z": 14})

    total_width = (len(shape_sequence) * shape_stride) - 4
    current_y = -(total_width / 2)

    for shape_name in shape_sequence:
        if shape_name not in SHAPES:
            print(f"Skipping unknown shape: '{shape_name}'")
            continue

        points.append(
            {
                "label": f"── {shape_name} {'(Mirrored)' if mirror else ''} ────────────────────────────────",
                "x": 13,
                "y": current_y * m_factor,
                "z": 14,
            }
        )

        strokes = SHAPES[shape_name]
        for i, stroke in enumerate(strokes):
            start_y = (stroke[0][0] + current_y) * m_factor
            start_z = stroke[0][1]
            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} travel",
                    "x": 13,
                    "y": start_y,
                    "z": start_z,
                }
            )
            points.append(
                {
                    "label": f"{shape_name} | stroke {i+1} pen-down",
                    "x": 16,
                    "y": start_y,
                    "z": start_z,
                }
            )

            for pt in stroke[1:]:
                py = (pt[0] + current_y) * m_factor
                pz = pt[1]
                points.append(
                    {"label": f"{shape_name} | draw", "x": 16, "y": py, "z": pz}
                )

            end_y = (stroke[-1][0] + current_y) * m_factor
            end_z = stroke[-1][1]
            points.append(
                {"label": f"{shape_name} | lift", "x": 13, "y": end_y, "z": end_z}
            )

        current_y += shape_stride

    points.append({"label": "Home-End", "x": 13, "y": 0, "z": 14})

    scene_name = "-".join(shape_sequence)
    payload = {
        "name": f"Draw {scene_name} {'(Mirrored)' if mirror else ''}",
        "type": "cartesian",
        "speed": 0.7,
        "_notes": [f"Draws: {', '.join(shape_sequence)}"],
        "points": points,
    }

    return json.dumps(payload, indent=2, sort_keys=False), scene_name


# --- INTERACTIVE MENU ---
def main():
    print("\n=== Robot Arm Path Generator ===")
    print("Available Shapes: TREE, CAR, HOUSE, STAR")
    print("Example input: TREE CAR STAR")

    # 1. Ask for shapes
    user_input = (
        input("\nEnter the shapes you want to draw (separated by spaces): ")
        .strip()
        .upper()
    )
    if not user_input:
        print("No shapes entered. Exiting.")
        sys.exit()

    shape_list = user_input.split()

    # 2. Ask for mirroring
    mirror_input = input("Do you want to mirror the output? (y/n): ").strip().lower()
    is_mirrored = mirror_input == "y"

    # 3. Generate the data
    json_output, scene_name = create_shapes_json(shape_list, mirror=is_mirrored)

    # 4. Save the file
    filename = f"{scene_name.lower()}{'_mirrored' if is_mirrored else ''}.json"
    with open(filename, "w") as f:
        f.write(json_output)

    print(f"\n✅ Success! Generated '{filename}' with {len(shape_list)} shape(s).")


if __name__ == "__main__":
    main()

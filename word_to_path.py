import json

# Font definition: Each letter is a list of "strokes".
# A stroke is a list of (y, z) coordinates. Width is 0-4, Height is 5-12.
ALPHABET = {
    "A": [[(0, 5), (2, 12), (4, 5)], [(1, 8.5), (3, 8.5)]],
    "B": [
        [(0, 12), (0, 5), (3, 5), (4, 6.5), (3, 8.5), (0, 8.5)],
        [(3, 8.5), (4, 10.5), (3, 12), (0, 12)],
    ],
    "C": [[(4, 10.5), (2, 12), (0, 10.5), (0, 6.5), (2, 5), (4, 6.5)]],
    "D": [[(0, 12), (0, 5), (2, 5), (4, 7), (4, 10), (2, 12), (0, 12)]],
    "E": [[(4, 12), (0, 12), (0, 5), (4, 5)], [(0, 8.5), (3, 8.5)]],
    "F": [[(4, 12), (0, 12), (0, 5)], [(0, 8.5), (3, 8.5)]],
    "G": [
        [(4, 10.5), (2, 12), (0, 10.5), (0, 6.5), (2, 5), (4, 6.5), (4, 8.5), (2, 8.5)]
    ],
    "H": [[(0, 12), (0, 5)], [(0, 8.5), (4, 8.5)], [(4, 12), (4, 5)]],
    "I": [[(0, 12), (4, 12)], [(2, 12), (2, 5)], [(0, 5), (4, 5)]],
    "J": [[(0, 6.5), (1, 5), (3, 5), (4, 6.5), (4, 12)]],
    "K": [[(0, 12), (0, 5)], [(4, 12), (0, 8.5), (4, 5)]],
    "L": [[(0, 12), (0, 5), (4, 5)]],
    "M": [[(0, 5), (0, 12), (2, 8.5), (4, 12), (4, 5)]],
    "N": [[(0, 5), (0, 12), (4, 5), (4, 12)]],
    "O": [[(2, 12), (4, 10.5), (4, 6.5), (2, 5), (0, 6.5), (0, 10.5), (2, 12)]],
    "P": [[(0, 5), (0, 12), (3, 12), (4, 10.5), (3, 8.5), (0, 8.5)]],
    "Q": [
        [(2, 12), (4, 10.5), (4, 6.5), (2, 5), (0, 6.5), (0, 10.5), (2, 12)],
        [(2, 7), (4, 5)],
    ],
    "R": [
        [(0, 5), (0, 12), (3, 12), (4, 10.5), (3, 8.5), (0, 8.5)],
        [(2, 8.5), (4, 5)],
    ],
    "S": [
        [
            (4, 10.5),
            (2, 12),
            (0, 10.5),
            (0, 9.5),
            (2, 8.5),
            (4, 7.5),
            (4, 6.5),
            (2, 5),
            (0, 6.5),
        ]
    ],
    "T": [[(0, 12), (4, 12)], [(2, 12), (2, 5)]],
    "U": [[(0, 12), (0, 6.5), (2, 5), (4, 6.5), (4, 12)]],
    "V": [[(0, 12), (2, 5), (4, 12)]],
    "W": [[(0, 12), (1, 5), (2, 8.5), (3, 5), (4, 12)]],
    "X": [[(0, 12), (4, 5)], [(4, 12), (0, 5)]],
    "Y": [[(0, 12), (2, 8.5), (4, 12)], [(2, 8.5), (2, 5)]],
    "Z": [[(0, 12), (4, 12), (0, 5), (4, 5)]],
    " ": [],
}


def create_mirrored_word_json(word, mirror=True):
    word = word.upper()
    points = []
    m_factor = -1 if mirror else 1

    points.append({"label": "Home", "x": 13, "y": 0, "z": 14})

    # Calculate starting Y
    total_width = (len(word) * 6) - 2
    current_y = -(total_width / 2)

    for char in word:
        if char not in ALPHABET:
            continue

        points.append(
            {
                "label": (
                    f"── {char} (Mirrored) ────────────────────────────────"
                    if mirror
                    else f"── {char} ──"
                ),
                "x": 13,
                "y": current_y * m_factor,
                "z": 14,
            }
        )

        strokes = ALPHABET[char]
        for i, stroke in enumerate(strokes):
            # Travel to start (Negate Y if mirrored)
            start_y = (stroke[0][0] + current_y) * m_factor
            start_z = stroke[0][1]
            points.append(
                {"label": f"{char} | travel", "x": 13, "y": start_y, "z": start_z}
            )
            points.append(
                {"label": f"{char} | pen-down", "x": 16, "y": start_y, "z": start_z}
            )

            # Draw strokes (Negate Y if mirrored)
            for pt in stroke[1:]:
                py = (pt[0] + current_y) * m_factor
                pz = pt[1]
                points.append({"label": f"{char} | draw", "x": 16, "y": py, "z": pz})

            points.append(
                {
                    "label": f"{char} | lift",
                    "x": 13,
                    "y": (stroke[-1][0] + current_y) * m_factor,
                    "z": stroke[-1][1],
                }
            )

        current_y += 6

    points.append({"label": "Home-End", "x": 13, "y": 0, "z": 14})

    payload = {
        "name": f"Write {word} {'(Mirrored)' if mirror else ''}",
        "type": "cartesian",
        "speed": 0.7,
        "points": points,
    }

    return json.dumps(payload, indent=2)


# Generate mirrored output
if __name__ == "__main__":
    print(create_mirrored_word_json("Hi", mirror=True))

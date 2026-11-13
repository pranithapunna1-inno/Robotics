"""
layouts.py - arena size and the selectable obstacle layouts.

All coordinates are in "arena pixels": (0, 0) is the top-left corner of the
arena and x grows to the right, y grows DOWN (standard screen convention).
Obstacles are axis-aligned rectangles written as (x, y, width, height).
"""

ARENA_W = 960
ARENA_H = 590

LAYOUTS = [
    {
        "name": "Scattered Walls",
        "start": (60, 530),
        "goal": (890, 70),
        "start_heading": 0,
        "obstacles": [
            (160, 380, 240, 40),
            (460, 200, 40, 300),
            (600, 120, 200, 40),
            (640, 330, 260, 40),
            (260, 150, 140, 40),
            (780, 460, 40, 130),
            (80, 220, 40, 150),
        ],
    },
    {
        "name": "Zig-Zag Corridor",
        "start": (50, 60),
        "goal": (900, 530),
        "start_heading": 90,
        "obstacles": [
            (220, 0, 40, 380),
            (420, 210, 40, 380),
            (620, 0, 40, 380),
            (800, 250, 160, 40),
            (100, 450, 120, 40),
        ],
    },
    {
        "name": "Obstacle Field",
        "start": (50, 295),
        "goal": (900, 295),
        "start_heading": 0,
        "obstacles": [
            (150, 180, 60, 60),
            (150, 350, 60, 60),
            (300, 90, 50, 200),
            (300, 380, 50, 160),
            (450, 220, 70, 70),
            (450, 420, 70, 70),
            (450, 60, 70, 70),
            (600, 150, 50, 260),
            (740, 60, 50, 250),
            (740, 390, 50, 150),
        ],
    },
]

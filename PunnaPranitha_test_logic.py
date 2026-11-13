"""
test_logic.py - headless tests (no window needed).  Run:  python test_logic.py

Needs pygame installed only because robot_sim.py imports it; no display is opened.
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import random

from layouts import ARENA_H, ARENA_W, LAYOUTS
from robot_sim import MODE_ORDER, Robot, Simulation


def test_layouts():
    for lay in LAYOUTS:
        assert len(lay["obstacles"]) >= 5, "need at least five obstacles"
        for (x, y, w, h) in lay["obstacles"]:
            assert 0 <= x and 0 <= y and x + w <= ARENA_W and y + h <= ARENA_H
    print("layouts OK")


def test_boundaries():
    r = Robot(50, 50, 180)          # facing the left wall
    for _ in range(200):
        r.move(10, [])
    assert r.x >= Robot.RADIUS - 1e-9
    r.heading = 270                 # facing the top wall
    for _ in range(200):
        r.move(10, [])
    assert r.y >= Robot.RADIUS - 1e-9
    print("boundaries OK")


def test_obstacle_blocks():
    obstacles = [(200, 0, 40, 400)]
    r = Robot(100, 100, 0)
    for _ in range(100):
        _, hit = r.move(10, obstacles)
    assert hit and r.x + Robot.RADIUS <= 200 + 1e-6
    print("obstacle blocking OK")


def test_differential_drive():
    # Straight line: equal wheel speeds -> heading unchanged.
    r = Robot(100, 300, 0)
    for _ in range(60):
        r.drive(1.0, 0.0, 1 / 60, 1.0, [])
    assert abs(r.heading) < 1e-6 and r.x > 150
    # Turn in place: wheels in opposite directions -> rotates, barely moves.
    r = Robot(400, 300, 0)
    for _ in range(60):
        r.drive(0.0, 1.0, 1 / 60, 1.0, [])
    assert r.heading > 100, "should have rotated clockwise"
    assert abs(r.x - 400) < 1 and abs(r.y - 300) < 1
    # Acceleration limit: wheels cannot jump to full speed in one frame.
    r = Robot(400, 300, 0)
    r.drive(1.0, 0.0, 1 / 60, 1.0, [])
    assert r.v_left < Robot.MAX_SPEED * 0.5
    print("differential drive OK")


def test_sensor_noise_and_filter():
    random.seed(1)
    r = Robot(100, 300, 0)
    obstacles = [(300, 0, 40, 600)]
    r.read_sensors(obstacles, noise=False)
    true_centre = r.sensors[2]
    r.read_sensors(obstacles, noise=True, reset_filter=True)
    errors = []
    for _ in range(200):
        r.read_sensors(obstacles, noise=True)
        assert all(0 <= v <= Robot.SENSOR_RANGE for v in r.sensors)
        errors.append(abs(r.sensors[2] - true_centre))
    mean_err = sum(errors) / len(errors)
    assert mean_err < Robot.NOISE_SIGMA, "filter should keep error below raw noise"
    print(f"sensor noise/filter OK (mean filtered error {mean_err:.1f}px)")


def run_to_goal(layout, mode, limit=120):
    sim = Simulation(layout)
    sim.set_mode(mode)
    t = 0.0
    while not sim.finished and t < limit:
        sim.update(1 / 60)
        t += 1 / 60
    return sim


def test_autonomous_modes_reach_goal():
    for seed in (1, 2, 3):                       # several random noise seeds
        random.seed(seed)
        for i in range(len(LAYOUTS)):
            for mode in ("auto", "reactive"):
                sim = run_to_goal(i, mode)
                assert sim.finished, f"seed {seed} layout {i + 1} {mode}: goal not reached"
                assert sim.collisions == 0, \
                    f"seed {seed} layout {i + 1} {mode}: {sim.collisions} collisions"
                if seed == 1:
                    print(f"layout {i + 1} {mode:8s} OK: {sim.elapsed:5.1f}s, "
                          f"{sim.distance / 10:5.0f} units, avoid events {sim.avoid_events}")
    print("auto + reactive reach the goal with 0 collisions (3 noise seeds)")


def test_assist_prevents_crash():
    random.seed(1)
    sim = Simulation(0)             # manual mode, assist on
    for _ in range(60 * 10):        # hold "forward" for 10 s
        sim.update(1 / 60, 1.0, 0.0)
    assert sim.collisions == 0, "assist should avoid repeated collisions"
    print(f"manual forward-drive with assist OK (avoid events {sim.avoid_events})")


def test_mode_cycle_and_records():
    sim = Simulation(0)
    seen = []
    for _ in range(3):
        sim.cycle_mode()
        seen.append(sim.mode)
    assert seen == ["auto", "reactive", "manual"]
    random.seed(1)
    sim = run_to_goal(0, "auto")
    assert (0, "auto") in sim.records
    print("mode cycling + records OK")


if __name__ == "__main__":
    test_layouts()
    test_boundaries()
    test_obstacle_blocks()
    test_differential_drive()
    test_sensor_noise_and_filter()
    test_autonomous_modes_reach_goal()
    test_assist_prevents_crash()
    test_mode_cycle_and_records()
    print("ALL TESTS PASSED")

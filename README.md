# Robotics
2D ROBOT NAVIGATION AND OBSTACLE AVOIDANCE SIMULATION
MegaMinds IT Services - Robotics Internship Assessment
Author: <YourName>

WHAT ARE NEEDED
- Python 3.9 or newer (I tested on Python 3.12)
- Pygame 2.1 or newer

HOW TO RUN (VS Code)
1. Unzip the folder and open it in VS Code (File > Open Folder).
2. Open the terminal (Ctrl + `). You can also make a virtual environment:
     python -m venv venv
     Windows:      venv\Scripts\activate
     macOS/Linux:  source venv/bin/activate
3. Install Pygame:
     pip install -r requirements.txt
4. Start the program:
     python robot_sim.py
   (on some systems use python3 or py)

TESTS (optional, no window needed)
     python test_logic.py

FILES
  robot_sim.py   Main program: robot, simulation logic, drawing, main loop
  navigation.py  Geometry, distance sensors, grid map and A* path planner
  layouts.py     Arena size and the 3 obstacle layouts
  test_logic.py  Checks for boundaries, collisions, A* and obstacle avoidance

CONTROLS
  Arrow keys / WASD   Drive (Up = forward, Down = reverse, Left/Right = turn)
  M                   Change mode: Manual -> A* -> Reactive
  H                   Turn avoidance assist on/off (manual mode)
  N                   Turn sensor noise on/off
  T                   Show/hide the A* path (or the goal line in reactive mode)
  V                   Show/hide sensor rays
  + / -               Change robot speed
  1 / 2 / 3 or L      Pick a layout / go to the next layout
  R                   Restart
  Space               Pause
  Esc                 Quit
  You can also click the on-screen buttons (Restart, Mode, Next layout).

FEATURES
- Arena with a START and a GOAL, and a visible robot with a heading arrow
- 5-10 obstacles in each of the 3 layouts
- The robot cannot leave the arena (its position is limited to the arena edges)
- Differential-drive robot: left and right wheels have their own speeds, so the
  robot turns in smooth curves
- Sensors: 5 distance rays and 2 side "whiskers", all visible. Sensor noise can
  be turned on, and a smoothing filter reduces it.
- Three modes:
  1. Manual - you drive with the keyboard. Optional assist slows the robot down
     and steers it toward the more open side.
  2. A* - the robot uses a map of the arena (obstacles are made bigger to leave
     a safe gap), finds the shortest path, and follows it. If it gets stuck, it
     backs up and plans again. Sensors act as a safety backup.
  3. Reactive - no map. The robot only uses its sensors and the goal direction.
     It heads to the goal, follows the obstacle edge when blocked, and leaves
     the edge when the way to the goal is clear.
- On-screen info: time, collisions, distance travelled, distance to goal,
  speed, avoidance events, noise on/off
- "GOAL REACHED!" message with a summary and a table comparing Manual, A* and
  Reactive results for the current layout
- Restart button, speed control, multiple layouts, pause

LIMITATIONS
- Obstacles are fixed rectangles. A* does not re-plan for moving obstacles.
- Reactive mode reaches the goal in all 3 layouts, but it is a simple method,
  so it may fail in other mazes. It is also slower than A*.
- Movement is simple: no wheel slip and no physics engine.
- In A* mode the path already avoids obstacles, so the sensors rarely need to
  act. Avoidance is easiest to see in manual and reactive modes.
- If assist is off in manual mode, hitting an obstacle just blocks the robot
  and counts as one collision.


# ArduPilot SITL: shared control

Human + AI shared control on an ArduPilot Copter running in SITL, using pymavlink.

## Setup
1. Start SITL: `cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --console`
2. Activate the venv: `source ~/venv-ardupilot/bin/activate`
3. Run scripts from a folder where you want the logs to be saved.

## scripts/
- test_drone.py: connect, arm, take off to 10 m, land
- test_move.py: fly north at 1 m/s using velocity setpoints
- test_shared.py: scripted human + AI, alpha blending (Eq. 22)
- test_keyboard.py: keyboard human-in-the-loop (w/a/s/d, q to quit)
- test_obstacle.py: virtual obstacle test. Run `python test_obstacle.py shared auto` and `python test_obstacle.py manual auto`
- telemetry_reader.py: telemetry reading

## plots/
Run from the folder that holds the CSV logs, for example:
`cd ardupilot/results && python ../plots/plot_obstacle.py`

## results/
| Run | Min distance to obstacle | Time inside obstacle |
|---|---|---|
| Human only | 1.0 m | 2.3 s |
| Shared control | 3.1 m | 0.0 s |

Same scripted human input in both runs. Collision limit is 2.0 m.

early_tests/ holds older keyboard and shared runs recorded before positions were made relative to the takeoff point, so their distance-to-target numbers are unreliable.

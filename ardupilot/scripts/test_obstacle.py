from pymavlink import mavutil
import numpy as np
import time, csv, sys, os, termios, tty, select

MODE = sys.argv[1] if len(sys.argv) > 1 else "shared"
AUTO = len(sys.argv) > 2 and sys.argv[2] == "auto"

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

m.set_mode("GUIDED"); time.sleep(1)
m.arducopter_arm(); m.motors_armed_wait()
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
while True:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if msg and -msg.z >= 9.5:
        break

START = np.array([msg.x, msg.y])   # takeoff point
BETA1, BETA2 = 2*np.pi/3, np.pi/2
OBS = np.array([15.0, 1.0])     # obstacle centre (north, east)
R_OBS = 2.0                     # obstacle radius (collision if closer)
D_SAFE = 7.0                    # AI starts avoiding inside this distance
D_HARD = 3.5                    # human cannot push toward obstacle inside this
target = np.array([30.0, 0.0])

def alpha_from(u_a, u_h):
    na, nh = np.linalg.norm(u_a), np.linalg.norm(u_h)
    if na < 1e-6 or nh < 1e-6:
        return 1.0
    eta = np.arccos(np.clip(u_a @ u_h / (na*nh), -1, 1))
    if eta >= BETA1: a = 0.0
    elif eta <= BETA2: a = 1.0
    else: a = (eta - BETA1) / (BETA2 - BETA1)
    s = min(na / 0.5, 1.0)
    return s * a + (1 - s) * 1.0

def ai_command(pos):
    goal = 0.5 * (target - pos)
    n = np.linalg.norm(goal)
    if n > 1.5: goal = goal * 1.5 / n
    d_vec = pos - OBS
    d = np.linalg.norm(d_vec)
    r_hat = d_vec / d if d > 1e-6 else np.array([-1.0, 0.0])
    f = np.clip((D_SAFE - d) / (D_SAFE - R_OBS), 0.0, 1.0)
    t_hat = np.array([-r_hat[1], r_hat[0]])
    gd = target - pos
    if t_hat @ gd < 0: t_hat = -t_hat
    u = goal + 2.5 * f * r_hat + 1.5 * f * t_hat
    n = np.linalg.norm(u)
    if n > 2.0: u = u * 2.0 / n
    return u, d

def send_velocity(vx, vy, vz):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

KEYS = {"w": (1.5, 0.0), "s": (-1.5, 0.0), "a": (0.0, -1.5), "d": (0.0, 1.5)}

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

print(f"\nMODE: {MODE}")
print("w=north  s=south  a=west  d=east  space=stop  q=quit+land")
print("Obstacle is 15 m north. Try pushing the drone straight into it.\n")

pos = np.array([0.0, 0.0]); alt = 10.0
u_h = np.zeros(2); human_until = 0.0
min_d = 999.0; coll_steps = 0
logname = f"obstacle_{MODE}_log.csv"
f = open(logname, "w", newline="")
w = csv.writer(f)
w.writerow(["t","north","east","alt","uh_n","uh_e","ua_n","ua_e","alpha","us_n","us_e","dist_obs"])

t0 = time.time(); last_print = 0
try:
    while time.time() - t0 < 150:
        t = time.time() - t0
        while select.select([sys.stdin], [], [], 0)[0]:
            k = os.read(fd, 1).decode(errors="ignore").lower()
            if k in KEYS:
                u_h = np.array(KEYS[k]); human_until = time.time() + 0.5
            elif k == " ":
                u_h = np.zeros(2); human_until = 0
            elif k == "q":
                raise KeyboardInterrupt
        if time.time() > human_until:
            u_h = np.zeros(2)
        if AUTO:
            u_h = np.array([1.5, 0.0])       # scripted human: always pushes north
            if pos[0] > 22 or t > 80:
                raise KeyboardInterrupt

        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg:
            pos = np.array([msg.x, msg.y]) - START; alt = -msg.z

        u_a, d = ai_command(pos)
        if MODE == "manual":
            a = 1.0
            u_s = u_h.copy()
        else:
            a = alpha_from(u_a, u_h)
            if d < D_HARD and u_h @ (OBS - pos) > 0:
                a = 0.0
            u_s = u_a + a * u_h
            n = np.linalg.norm(u_s)
            if n > 2.0: u_s = u_s * 2.0 / n
        send_velocity(u_s[0], u_s[1], 0.0)

        min_d = min(min_d, d)
        if d < R_OBS: coll_steps += 1
        w.writerow([round(t,2), pos[0], pos[1], alt, *u_h, *u_a, a, *u_s, d])
        if t - last_print >= 1:
            print(f"t={t:4.0f}s north={pos[0]:5.1f} east={pos[1]:5.1f} dist_obs={d:4.1f} alpha={a:.2f}")
            last_print = t
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    f.close()
    m.set_mode("LAND")
    print(f"\nMinimum distance to obstacle centre: {min_d:.1f} m (collision if below {R_OBS} m)")
    print(f"Time inside obstacle: {coll_steps*0.1:.1f} s")
    print(f"Landing. Log saved to ~/{logname}")

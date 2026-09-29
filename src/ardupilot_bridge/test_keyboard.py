from pymavlink import mavutil
import numpy as np
import time, csv, sys, os, termios, tty, select

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

BETA1, BETA2 = 2*np.pi/3, np.pi/2

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

def send_velocity(vx, vy, vz):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

KEYS = {"w": (1.0, 0.0), "s": (-1.0, 0.0), "a": (0.0, -1.0), "d": (0.0, 1.0)}

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

print("\nKEYBOARD CONTROL:  w=north  s=south  a=west  d=east  space=stop  q=quit+land")
print("The AI is flying toward a target 30 m north.\n")

target = np.array([30.0, 0.0])
pos = np.array([0.0, 0.0]); alt = 10.0
u_h = np.zeros(2); human_until = 0.0
f = open("keyboard_log.csv", "w", newline="")
w = csv.writer(f)
w.writerow(["t","north","east","alt","uh_n","uh_e","ua_n","ua_e","alpha","us_n","us_e"])

t0 = time.time(); last_print = 0
try:
    while time.time() - t0 < 90:
        t = time.time() - t0
        # read keys
        while select.select([sys.stdin], [], [], 0)[0]:
            k = os.read(fd, 1).decode(errors="ignore").lower()
            if k in KEYS:
                u_h = np.array(KEYS[k]); human_until = time.time() + 0.4
            elif k == " ":
                u_h = np.zeros(2); human_until = 0
            elif k == "q":
                raise KeyboardInterrupt
        if time.time() > human_until:
            u_h = np.zeros(2)

        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg:
            pos = np.array([msg.x, msg.y]); alt = -msg.z

        u_a = 0.5 * (target - pos)
        n = np.linalg.norm(u_a)
        if n > 1.5: u_a = u_a * 1.5 / n

        a = alpha_from(u_a, u_h)
        u_s = u_a + a * u_h
        n = np.linalg.norm(u_s)
        if n > 2.0: u_s = u_s * 2.0 / n
        send_velocity(u_s[0], u_s[1], 0.0)

        w.writerow([round(t,2), pos[0], pos[1], alt, *u_h, *u_a, a, *u_s])
        if t - last_print >= 1:
            print(f"t={t:4.0f}s  north={pos[0]:5.1f}  east={pos[1]:5.1f}  alpha={a:.2f}  human={u_h}\r")
            last_print = t
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    f.close()
    m.set_mode("LAND")
    print("Landing. Log saved to ~/keyboard_log.csv")

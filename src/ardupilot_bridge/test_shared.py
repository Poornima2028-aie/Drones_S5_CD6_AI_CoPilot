from pymavlink import mavutil
import numpy as np
import time, csv

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
print("At 10 m. Starting shared control...")

BETA1, BETA2 = 2*np.pi/3, np.pi/2

def alpha_from(u_a, u_h):
    na, nh = np.linalg.norm(u_a), np.linalg.norm(u_h)
    if na < 1e-6 or nh < 1e-6:
        return 1.0
    eta = np.arccos(np.clip(u_a @ u_h / (na*nh), -1, 1))
    if eta >= BETA1: return 0.0
    if eta <= BETA2: return 1.0
    return (eta - BETA1) / (BETA2 - BETA1)

def send_velocity(vx, vy, vz):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

target = np.array([30.0, 0.0])
pos = np.array([0.0, 0.0]); alt = 10.0
f = open("shared_log.csv", "w", newline="")
w = csv.writer(f)
w.writerow(["t","north","east","alt","uh_n","uh_e","ua_n","ua_e","alpha","us_n","us_e"])

t0 = time.time(); last_print = 0
while time.time() - t0 < 30:
    t = time.time() - t0
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
    if msg:
        pos = np.array([msg.x, msg.y]); alt = -msg.z

    # autonomous command: go toward target, max 1.5 m/s
    u_a = 0.5 * (target - pos)
    n = np.linalg.norm(u_a)
    if n > 1.5: u_a = u_a * 1.5 / n

    # simulated human
    if t < 10:   u_h = np.array([1.0, 0.0])       # same direction
    elif t < 20: u_h = np.array([-1.0, 0.0])      # opposite direction
    else:        u_h = np.array([-0.26, 0.97])    # sideways-ish

    a = alpha_from(u_a, u_h)
    u_s = u_a + a * u_h
    n = np.linalg.norm(u_s)
    if n > 2.0: u_s = u_s * 2.0 / n
    send_velocity(u_s[0], u_s[1], 0.0)

    w.writerow([round(t,2), pos[0], pos[1], alt, *u_h, *u_a, a, *u_s])
    if t - last_print >= 1:
        print(f"t={t:4.1f}s  north={pos[0]:5.1f}  east={pos[1]:5.1f}  alpha={a:.2f}")
        last_print = t
    time.sleep(0.1)

f.close()
m.set_mode("LAND")
print("Landing. Log saved to ~/shared_log.csv")

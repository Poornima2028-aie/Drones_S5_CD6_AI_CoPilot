from pymavlink import mavutil
import time

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
print("Connected!")
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

m.set_mode("GUIDED")
time.sleep(1)
m.arducopter_arm()
m.motors_armed_wait()
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

# wait until we reach about 10 m
while True:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if msg and -msg.z >= 9.5:
        break
print("At 10 m, now moving north...")

def send_velocity(vx, vy, vz):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

t0 = time.time()
while time.time() - t0 < 10:
    send_velocity(1.0, 0.0, 0.0)      # 1 m/s north
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
    if msg:
        print(f"North: {msg.x:.1f} m   Altitude: {-msg.z:.1f} m")
    time.sleep(0.1)

m.set_mode("LAND")
print("Landing")

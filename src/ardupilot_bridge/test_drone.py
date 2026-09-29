from pymavlink import mavutil
import time

print("Waiting for drone...")
m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
print("Connected!")

m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

m.set_mode("GUIDED")
time.sleep(1)
m.arducopter_arm()
m.motors_armed_wait()
print("Armed")

m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

t0 = time.time()
while time.time() - t0 < 20:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if msg:
        print(f"Altitude: {-msg.z:.1f} m")

m.set_mode("LAND")
print("Landing")

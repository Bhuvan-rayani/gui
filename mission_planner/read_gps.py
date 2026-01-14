from pymavlink import mavutil
import time

drone1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
drone2 = mavutil.mavlink_connection('udp:127.0.0.1:14560')

drone1.wait_heartbeat()
drone2.wait_heartbeat()

while True:
    msg1 = drone1.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    msg2 = drone2.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

    if msg1:
        lat1 = msg1.lat / 1e7
        lon1 = msg1.lon / 1e7
        alt1 = msg1.relative_alt / 1000.0

    if msg2:
        lat2 = msg2.lat / 1e7
        lon2 = msg2.lon / 1e7
        alt2 = msg2.relative_alt / 1000.0

    time.sleep(0.05)

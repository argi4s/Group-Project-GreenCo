from finalSITL import DroneController

drone = DroneController()

while True:

    cmd = input("> ")

    if cmd == "status":
        drone.status()

    elif cmd == "arm":
        drone.arm_with_ack_timeout()

    elif cmd == "guided":
        drone.set_mode_ack("GUIDED")

    elif cmd == "wp":
        drone.next_waypoint()

    elif cmd == "exit":
        drone.stop()
        break
import redis

my_msg_broker = redis.Redis(decode_responses=True)
robo_name = "/LineTracer1"
MY_COMM_CH = "COMMANDS"

while True:
    command = int(
        input(
            "Enter your command:: \n 1. Park AV1  \n 2. Park AV2 \n 3. Park Both  \n 4. Emergency Inner Lane  \n 5. Emergency Outer Lane \n 6. Restart AV1 \n 7. Restart AV2\n 8. Emergency Over\n 9. Simulate Emergency in AV1\n 10. Simulate Emergency in AV2\n 11. u-turn Av1\n 12. u-turn AV2\n"
        )
    )

    if command == 1:
        robo_name = "/LineTracer1"
        cmd = 'parking_cmd'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Parking command send to vehicle 1.")

    elif command == 2:
        robo_name = "/LineTracer2"
        cmd = 'parking_cmd'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Parking command send to vehicle 2.")

    elif command == 3:
        cmd = 'parking_cmd'

        robo_name = "/LineTracer1"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)

        robo_name = "/LineTracer2"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)

        print("Parking command send to both vehicle.")

    elif command == 4:
        cmd = 'emergency_in_inner_lane'
        robo_name = "/LineTracer1"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Inform vehicle 1: emergency in inner lane")

        robo_name = "/LineTracer2"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Inform vehicle 2: emergency in inner lane")


    elif command == 5:
        cmd = 'emergency_in_outer_lane'
        robo_name = "/LineTracer1"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Inform vehicle 1: emergency in outer lane")

        robo_name = "/LineTracer2"
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Inform vehicle 2: emergency in outer lane")

    elif command == 6:
        robo_name = "/LineTracer1"
        cmd = 'restart_from_parking'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Restart vehicle 1")

    elif command == 7:
        robo_name = "/LineTracer2"
        cmd = 'restart_from_parking'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Restart vehicle 2")

    elif command == 8:
        cmd = 51
        print("Emergency over")

    elif command == 9:
        robo_name = "/LineTracer1"
        cmd = 'emergency'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Simulate Emergency in AV1")

    elif command == 10:
        robo_name = "/LineTracer2"
        cmd = 'emergency'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Simulate Emergency in AV2")

    elif command == 11:
        robo_name = "/LineTracer1"
        cmd = 'U-turn'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Simulate U-turn in AV1")

    elif command == 12:
        robo_name = "/LineTracer2"
        cmd = 'U-turn'
        my_msg_broker.publish(robo_name + "/" + MY_COMM_CH, cmd)
        print("Simulate U-turn in AV2")

    else:
        print("Please enter a valid commmand (1-10)")

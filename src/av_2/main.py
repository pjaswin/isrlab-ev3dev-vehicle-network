#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.messaging import BluetoothMailboxClient, NumericMailbox
from pybricks.media.ev3dev import Font, SoundFile
import random

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the ultrasonic sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S4)

# Initialise touch sensor
crash_sensor = TouchSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=118)
ev3 = EV3Brick()
# Current speed, normal speed, maximum speed, and minimum speed
DRIVE_SPEED = 70
NORMAL_SPEED = 70
MAX_SPEED = 100
MIN_SPEED = 60

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 10
WHITE = 65
threshold = (BLACK + WHITE) / 2

# Set the gain of the PID controssller.
# PROPORTIONAL_GAIN = 0.4
# INTEGRAL_GAIN = 0.1
# DERIVATIVE_GAIN = 0.5

PROPORTIONAL_GAIN = 0.4
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.5


# Color Codes

STOP = Color.RED
INTERSECTION = "GREEN"  # TODO: To be changed
# PARKING_AV = Color.RED
PARKING_AV = "BLUE"  # BLUE
PARKING_SPCL = "YELLOW"  # YELLOW


def detectable_colors(rgb):
    # check for blue
    if rgb[2] >= 65:
        if rgb[0] <= 15:
            if rgb[1] <= 30:
                return "BLUE"

    # check for yellow
    elif rgb[0] >= 40 and rgb[0] <= 54:
        if rgb[1] >= 25 and rgb[1] <= 35:
            if rgb[2] >= 3 and rgb[2] <= 15:  # was 8 for av1
                return "YELLOW"
    # check for green
    elif rgb[1] >= 25:
        if rgb[0] <= 10:
            if rgb[2] <= 17:
                return "GREEN"


# Intialize variables related to PID controller.
deviation = 0.0
derivative = 0.0
integral = 0.0
last_deviation = 0.0
turn_rate = 0.0

# Lane change state
# step 0: drive on the outer lane
# step 1: change lane (from outer to inner)
# step 2: drive on the inner lane
# step 3: change lane (from inner to outer)
step = 0
# Time that state changed previously
previousStateChangedTime = 0
# Time that robot has stopped
stop_time = 0
# The flag to control robot stop or not
stopping = False

# The flag for emergency vehicle to arrive
sos = False
# data = DataLog('time', 'step', 'color', 'speed', 'distance', 'stop', 'deviation', 'integral', 'derivative')

# Initialize Bluetooth client and mailboxes for message passing.
client = BluetoothMailboxClient()
mbox_id = NumericMailbox("id2", client)
mbox_time = NumericMailbox("time2", client)
mbox_lane = NumericMailbox("lane2", client)
mbox_speed = NumericMailbox("speed2", client)
mbox_distance = NumericMailbox("distance2", client)
mbox_parking = NumericMailbox("parking2", client)  # from server
mbox_emergency = NumericMailbox("emergency2", client)  # from server
mbox_crash = NumericMailbox("crash2", client)
mbox_server_ack = NumericMailbox("ack2", client)  # TODO: Do we need this? #from server


# Server robot name
SERVER = "C8:E2:65:CD:69:86"
print("establishing connection...")
client.connect(SERVER)
print("server connected!")

# # Wait until receive message from the negotiator.
while True:
    msg_id = mbox_id.read()
    if msg_id != None:
        break

# Start time.
watch = StopWatch()
watch.reset()

big_font = Font(size=100, bold=True)
ev3.screen.set_font(big_font)

# Added vars for testing
crash = False
emergency = False
park = 0

alert_mode = False
while True:
    # em_int = random.randint(0, 150)
    # park_int = random.randint(0, 150)
    ev3.screen.draw_text(60, 50, "AV2")

    time = watch.time()
    reflection = line_sensor.reflection()
    color = line_sensor.color()
    rgb = line_sensor.rgb()
    distance = obstacle_sensor.distance()
    crash = crash_sensor.pressed()

    # Send messages to the SERVER.
    mbox_id.send(2)
    mbox_time.send(time)
    mbox_lane.send(step)
    mbox_speed.send(DRIVE_SPEED)
    mbox_distance.send(distance)
    mbox_parking.send(park)

    # Read messages from the SERVER
    park_command = mbox_parking.read()
    emergency = mbox_emergency.read()

    if emergency:
        emergency_resp = 100
        ev3.light.on(Color.RED)
        print("//////////////////// CAUTION !!! /////////////////////")
        print("//////////////////////LANE:", emergency % 10, "/////////////////////")
        mbox_emergency.send(emergency_resp)
    else:
        emergency_resp = 300
        mbox_emergency.send(emergency_resp)
        ev3.light.on(Color.GREEN)
        alert_mode = False

    if park_command == 999 and park != 200:
        park = 200
        mbox_parking.send(park)  # TODO: Status code for recieving parking signal
    elif park_command == 666 and park:
        park = 222
        mbox_parking.send(park)
    # emergency = mbox_emergency.read()  # TODO: Need to decide the type of message

    # Case 1: Vehicle crashed
    # if em_int == -73:
    #     ev3.speaker.beep()
    #     ev3.light.on(Color.YELLOW)
    #     emergency = 10

    if crash:
        sos = True

    if sos:
        robot.stop()
        ev3.light.on(Color.RED)
        stopping = True
        # DRIVE_SPEED = 0
        while mbox_server_ack.read() != 200:
            ev3.speaker.beep()
            mbox_id.send(2)
            mbox_time.send(time)
            mbox_lane.send(step)
            mbox_speed.send(0)
            mbox_distance.send(0)
            mbox_crash.send(10 + step)
            print("-==-=-=-=-=-=-=-=-VEHICLE CRASHED. SOS TO SERVER=-=-=-=-=-=")

        print(
            "####################### WAITING FOR EMERGENCY VEHICLE ########################"
        )
        # Crash message only send until the server acknowledges
    else:
        if step == 1:
            if (
                reflection > 55
                or color == Color.WHITE
                or color == Color.YELLOW  # was WHITE
            ) and time >= previousStateChangedTime + 2000:
                integral = 0.0
                derivative = 0.0
                step = 2
                previousStateChangedTime = time
        elif step == 3 and time >= previousStateChangedTime + 2000:
            if reflection > 55 or color == Color.WHITE or color == Color.YELLOW:
                integral = 0.0
                derivative = 0.0
                step = 0
                previousStateChangedTime = time

        # Case 2: If Red light/STOP sign detected
        if color == STOP:
            robot.stop()

        # Case 3: Emergency state
        elif (
            emergency and not alert_mode
        ):  # if no emergency then server should send 0, else send 10 or 11 - emergency in lane 0 or emergency in lane 1
            # If an obstable is detected, robot stops 1000ms and does lane change
            # Only when the robot is driving on the lane
            if distance > 50:  # useless condition
                if (
                    not stopping
                    and time >= stop_time + 1000
                    and step != 1
                    and step != 3
                ):
                    stopping = True
                    stop_time = time
                else:
                    if time >= stop_time + 1000:
                        # if step == 0 or step == 2:
                        if step == emergency % 10:
                            print(
                                "\n################# Going into Alert Mode ##################\n"
                            )
                            step = step + 1
                            previousStateChangedTime = time
                        alert_mode = True
                        integral = 0.0
                        derivative = 0.0
                        stopping = False
                        stop_time = time
            else:
                stopping = False
                stop_time = time

        # if not stopping and step != 1 and step != 3:
        #     # stopping = True
        #     stop_time = time
        # else:
        #     if step == emergency % 10:
        #         step += 1
        #         alert_mode = True
        #         previousStateChangedTime = time
        #     integral = 0.0
        #     derivative = 0.0
        #     stopping = False
        #     stop_time = time

        #     DRIVE_SPEED = MIN_SPEED
        # Server should broadcast emergency message for a fixed duration or until the emergency is resolved, whichever is the best TODO!

        # Case 4: Parking (No emergency and  parking)
        elif park == 200 and not stopping and distance > 245:
            if detectable_colors(rgb) == PARKING_AV:  # Decide colors!! Parking color
                print(
                    "+++++++++++++++++++++++Parking spot detected++++++++++++++++++++++++"
                )
                robot.stop()
                if step == 0:
                    angle = -1
                elif step == 2:
                    angle = 1

                robot.turn(angle * 90)
                wait(2000)
                distance = obstacle_sensor.distance()

                print("\nDistance ahead: ", distance)
                print("")
                if distance < 200:  # TODO! Calibrate the values
                    print(
                        "+++++++++++Parking lot occupied. Search for another spot.+++++++++++"
                    )

                    robot.turn(-angle * 90)
                    robot.straight(100)
                else:
                    robot.straight(200)
                    robot.turn(-angle * 90)
                    print(
                        "++++++++++++++++++++++.Vehicle parked.+++++++++++++++++++++++++++++"
                    )
                    stopping = True
                    ev3.speaker.play_file(SoundFile.MOTOR_STOP)
                    step += 4  # Check the impact of this. To denote parking. NEED TO STORE PREVIOUS STATE. maybe add some constant to step instead of storing the prev state
                    park = 200
                    mbox_parking.send(park)
                    DRIVE_SPEED = 0

            elif (
                detectable_colors(rgb) == INTERSECTION
                or detectable_colors(rgb) == PARKING_SPCL
            ):
                robot.straight(50)

        # Case 4b: Parking Over. Return to track.
        elif park == 222 and step > 3:
            park = 0
            step -= 4
            if step == 0:
                angle = -1
            elif step == 2:
                angle = 1

            ev3.speaker.play_file(SoundFile.MOTOR_START)
            robot.turn(-angle * 90)
            robot.straight(200)
            robot.turn(angle * 90)
            stopping = False
            DRIVE_SPEED = MIN_SPEED

        # Case 5: No server or physical interuptions (Just lane following)
        elif not stopping and (
            detectable_colors(rgb) == PARKING_AV
            or detectable_colors(rgb) == PARKING_SPCL
            or detectable_colors(rgb) == INTERSECTION
        ):
            robot.straight(50)

        # Case 6: Obstacle detected
        elif distance <= 300 and step < 4:  # DISABLED!!
            ev3.light.on(Color.YELLOW)
            # robot.stop()
            print(
                "------------------------- Obstacle Detected !! ------------------------"
            )
            if not stopping and time >= stop_time + 1000 and step != 1 and step != 3:
                stopping = True
                stop_time = time
            else:
                if time >= stop_time + 1000:
                    if step == 0 or step == 2:
                        step = step + 1
                        previousStateChangedTime = time
                    integral = 0.0
                    derivative = 0.0
                    stopping = False
                    stop_time = time
        elif not park:
            stopping = False
            stop_time = time

        if step == 1 or step == 3 or color == STOP:
            DRIVE_SPEED = MIN_SPEED
        elif distance > 600:
            if DRIVE_SPEED < MAX_SPEED:
                DRIVE_SPEED = DRIVE_SPEED + 0.5
        elif distance > 550 and distance <= 600:
            DRIVE_SPEED = NORMAL_SPEED
        elif distance > 500 and distance <= 550:
            if DRIVE_SPEED > MIN_SPEED:
                DRIVE_SPEED = DRIVE_SPEED - 1
        else:
            DRIVE_SPEED = MIN_SPEED

        if not stopping and color != STOP:
            # Calculate the deviation from the threshold.
            deviation = reflection - threshold
            # Calculate the integral.
            if (
                detectable_colors(rgb) == INTERSECTION
                or detectable_colors(rgb) == PARKING_AV
                and not park
            ):
                # robot.settings(DRIVE_SPEED, 0,0,0)
                robot.straight(50)

            elif deviation > -10 and deviation < 10:
                integral = 0
            elif deviation * last_deviation < 0:
                integral = 0
            else:
                integral = integral + deviation
            # Calculate the derivative.
            derivative = deviation - last_deviation

            # Calculate the turn rate.
            turn_rate = (
                (PROPORTIONAL_GAIN * deviation)
                + (INTEGRAL_GAIN * integral)
                + (DERIVATIVE_GAIN * derivative)
            )

            # step 0: use the calculated turn_rate
            # step 1: robot is turning right
            # step 2: use the opposite calculated turn_rate
            # step 3: robot is turning left
            if step == 1:
                turn_rate = 12
            elif step == 2:
                turn_rate = -1 * turn_rate
            elif step == 3:
                turn_rate = -14

            # Set the drive base speed and turn rate.
            robot.drive(DRIVE_SPEED, turn_rate)

            last_deviation = deviation

        else:
            robot.stop()

        end_time = watch.time()

        print("__________________________")
        print("step: " + str(step))
        print("park: ", park)
        print("emergency: ", emergency)
        # print("crash: ", crash)
        print("color: " + str(color))
        print("reflection: " + str(reflection))
        print("speed: " + str(DRIVE_SPEED))
        print("distance: " + str(distance))
        print("turn rate: " + str(turn_rate))
        # print("stopOrNot: " + str(stopping))
        # print("stop time: "+str(stop_time))
        # print("time: "+str(time))
        # print("time difference: "+str(end_time-time))
        print("__________________________")

        # Store data log.
        # data.log(time, step, color, DRIVE_SPEED, distance, stopping, deviation, integral, derivative)

        # Keep time of each loop constant 100ms.
        wait_time = 0
        if (end_time - time) < 100:
            wait_time = 100 - (end_time - time)

# Open questions:
# 1) How to handle emergency when a parking request is generated
# 2) How to handle emergency when a vehicle is making a turn_rate
# 3) How to handle emergency when an obstacle is detected
# 4) The lane number of a vehicle joining the lane should be 0 or should it be a different number (as it can affect during an emergency)

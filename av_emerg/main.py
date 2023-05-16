#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.messaging import BluetoothMailboxClient, NumericMailbox
from pybricks.media.ev3dev import Font, SoundFile
import random

ev3=EV3Brick()
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the ultrasonic sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S4)
#Initialise touch sensor
# crash_sensor = TouchSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=118)

# Current speed, normal speed, maximum speed, and minimum speed
DRIVE_SPEED = 60
NORMAL_SPEED = 60
MAX_SPEED = 100
MIN_SPEED = 40

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 5 #GREEN
WHITE = 50
threshold = (BLACK + WHITE) / 2

# Set the gain of the PID controller.
PROPORTIONAL_GAIN = 0.4
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.5

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

# data = DataLog('time', 'step', 'color', 'speed', 'distance', 'stop', 'deviation', 'integral', 'derivative')

# Initialize Bluetooth client and mailboxes for message passing.
client = BluetoothMailboxClient()
embox_id = NumericMailbox('id2', client)
embox_time = NumericMailbox('time2', client)
embox_lane = NumericMailbox('lane2', client)
embox_speed = NumericMailbox('speed2', client)
embox_distance = NumericMailbox('distance2', client)
embox_emer=NumericMailbox("emer2",client)
embox_emergencyover = NumericMailbox("emergency1", client)
# # mbox_parking = NumericMailbox('parking2', client)
# #
# # # Server robot name
SERVER = 'C8:E2:65:CD:69:86'
print('establishing connection...')
client.connect(SERVER)
print('server connected!')
# #
# # # Wait until receive message from the negotiator.
while True:
    msg_id = embox_id.read()
    if msg_id != None:
        break

# Start time.
watch = StopWatch()
watch.reset()
emergency=0
emerg_detected=0 #emerg_detected
emer_sit=0
park=0
lane=0
emerg=10
light=0
direction=0#to understand position
big_font=Font(size=100,bold=True)
ev3.screen.set_font(big_font)


def detectable_colors(rgb):
# check for blue
    if rgb[2] >= 65:
        if rgb[0] <= 15:
            if rgb[1] <= 30:
                return "BLUE"
# check for yellow

    elif rgb[0] >= 45 and rgb[0] <= 60:
        if rgb[1] >= 25 and rgb[1] <= 42:
            if rgb[2] >= 10 and rgb[2] <= 25: # was 8 for av1
                    return "YELLOW"
# check for green
    elif rgb[1] >= 25:
        if rgb[0] <= 10:
            if rgb[2] <= 25:
                return "GREEN"

while True:
    ev3.screen.draw_text(60,50,"EMERG")
    emerg=embox_emer.read()
    if emerg==10 or emerg==12:
        embox_emer.send(100)
        embox_emer.send(100)
        embox_emer.send(100)
        embox_emer.send(100)
        time = watch.time()
        color = line_sensor.reflection()
        real_color = line_sensor.color()
        distance = obstacle_sensor.distance()
        rgb=line_sensor.rgb()
        list1=emerg%10
        print(list1)
        if emerg_detected==0:#emerg_detected
            direction==0
            robot.reset()
            ev3.speaker.play_file(SoundFile.MOTOR_START)
            emergency=1
            park=0
            emerg_detected=1#emerg_detected
            DRIVE_SPEED = 60
            stopping = False
            watch = StopWatch()
            watch.reset()
            lane=list1
            cotime = watch.time()
            turn_angle=1
            robot.turn(90 * turn_angle)
            if lane==0:
                while line_sensor.color()!=Color.BLACK:
                    robot.drive(DRIVE_SPEED, 0)
                robot.straight(50)
                robot.turn(-80*turn_angle) #change -
                robot.stop()
                step=lane
                # robot.straight(150)
                # robot.turn(-90*turn_angle)
                # step=3
            else:
                while line_sensor.color()!=Color.BLACK:
                    robot.drive(DRIVE_SPEED, 0)
            # robot.straight(350)
                robot.straight(200)
                robot.turn(-90*turn_angle)
                step=lane
                # robot.straight(350)
                # robot.turn(-90*turn_angle)
                # step=1
        track_colors=detectable_colors(rgb)
        print(DRIVE_SPEED)
        if track_colors== "GREEN" and emer_sit==1:
            robot.stop()
            turn_angle=1
            if (direction==0):
                turn_angle=-1
            robot.turn(90*turn_angle)
            if lane == 2:
                robot.straight(400)
            else:
                robot.straight(200)
            robot.turn(90)
            step=4
            stopping = True
            DRIVE_SPEED = 0
            park=1
            emerg=0
            emer_sit=0
            emerg_detected=0#emerg_detected
            embox_emergencyover.send(0)
            ss=embox_emergencyover.read()
            while ss!=200:
                embox_emergencyover.send(0)
                ss=embox_emergencyover.read()
            while emerg!=99:
                emerg=embox_emer.read()
        if step == 1:
            if color > 65 and time >= previousStateChangedTime+2000:
                integral = 0.0
                derivative = 0.0
                step = 2
                previousStateChangedTime = time
        elif step == 3:
            if color > 65 and time >= previousStateChangedTime+2000:
                integral = 0.0
                derivative = 0.0
                step = 0
                previousStateChangedTime = time

        # # If an obstable is detected, robot stops 1000ms and does lane change
        # # Only when the robot is driving on the lane
        # print("STOP TIME:",stop_time)
        if distance <= 30 and emergency == 1:  # was500
            if not stopping and time >= stop_time+1000 and step != 1 and step != 3:
                stopping = True
                stop_time = time
            else:
                if time >= stop_time+1000:
                    if step == 0 or step == 2:
                        step = step + 1
                        previousStateChangedTime = time
                    integral = 0.0
                    derivative = 0.0
                    stopping = False
                    stop_time = time
        elif emergency==1:
            stopping = False
            stop_time = time
        listofcolor=[Color.RED,Color.BLUE,Color.YELLOW]
        if emergency==1:
            # ev3.speaker.play_file(SoundFile.HORN_1)
            if light<3:
             ev3.light.on(listofcolor[light])
             light=+1
            else:
             light=0
        # # While changing lane, keep the minimum speed
        # # Based on distance, increase or decrease speed 
        if step == 1 or step == 3:
            DRIVE_SPEED = MIN_SPEED
        elif emergency==1 and (watch.time()-cotime)>10000:
            if DRIVE_SPEED < MAX_SPEED:
                DRIVE_SPEED = DRIVE_SPEED+0.3
        elif distance > 600:
            if DRIVE_SPEED < MAX_SPEED:
                DRIVE_SPEED = DRIVE_SPEED + 0.3
        elif distance > 550 and distance <= 600:
            DRIVE_SPEED = NORMAL_SPEED
        elif distance > 500 and distance <= 550:
            if DRIVE_SPEED > MIN_SPEED:
                DRIVE_SPEED = DRIVE_SPEED - 1
        else:
            DRIVE_SPEED = MIN_SPEED

        if distance<180 and emergency==1:
            robot.stop()
            wait(5000)
            emer_sit=1
            if(robot.distance()<2500):
                direction=1
                if lane==1:
                    robot.turn(180)
                else:
                    robot.turn(-180)
                
                DRIVE_SPEED = MIN_SPEED
                if step==0:
                    step=2
                else:
                    step=0

        if not stopping and emergency==1 and park==0:
            # Calculate the deviation from the threshold.
            deviation = color - threshold
                
            if deviation > -10 and deviation < 10:
                integral = 0
            elif deviation * last_deviation < 0:
                integral = 0
            else:
                integral = integral + deviation
            # Calculate the derivative.
            derivative = deviation - last_deviation

        #     # Calculate the turn rate.
            turn_rate = (PROPORTIONAL_GAIN * deviation) + (INTEGRAL_GAIN * integral) + (DERIVATIVE_GAIN * derivative)
            
            if step == 1:
                turn_rate = 12
            elif step == 2:
                turn_rate = -1*turn_rate
            elif step == 3:
                turn_rate = -14

        #     # Set the drive base speed and turn rate.
            robot.drive(DRIVE_SPEED, turn_rate)

            last_deviation = deviation

        else: 
            robot.stop()

        end_time = watch.time()
        
        wait_time = 0
        if (end_time-time) < 100:
            wait_time = 100-(end_time-time)
    # wait(wait_time)
	
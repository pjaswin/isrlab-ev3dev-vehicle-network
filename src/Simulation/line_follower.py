"""
    Implementation of the Body class of the Agent
"""

import logging
from zmqRemoteApi import RemoteAPIClient
from typing import Any, List
import time
import redis
import threading

MY_PERC_CH = "PERCEPTIONS"  # from the physical body to the virtual body and controller
MY_COMM_CH = "COMMANDS"  # from virtual body to physical
FREE_SPACE = 1
ROTATION_DELAY = 0.05
MY_ABILITIES = ['left', 'forward', 'right', 'stop', 'none', 'slight_left', 'slight_right', 'stop_on_red', 'lane_change', 'parking', 'restart_from_parking', 'emergency_in_outer_lane', 'emergency_in_inner_lane', 'emergency', 'U-turn', 'parking_emergency', 'parking_cmd']


# Set the gain of the PID controller.
PROPORTIONAL_GAIN = 0.4
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.5

# Intialize variables related to PID controller.
deviation = 0.0
derivative = 0.0
integral = 0.0
last_deviation = 0.0

class Body:
    _my_sensors: List
    _my_actuators: List
    _my_name: str
    _my_perceptions: set
    _direction: str

    def __init__(self, name, robot_sens_array, robot_act_array):
        logging.info(f"Agent name: {name} initialized.")
        self._my_name = name
        self._my_sensors = robot_sens_array
        self._my_actuators = robot_act_array

    def __repr__(self):
        out = f"{self._my_name}: I have {len(self._my_sensors)} sensors - {self._my_sensors} and {len(self._my_actuators)} list of actuators {self._my_actuators }"
        return out

    def perceive(self, stimuli):
        # transform stimuli from sensors into perceptions
        for s in stimuli:
            self._my_perceptions.add(...)
        pass

    def step(self, t: int):
        # called by the simulator at each simulation step
        logging.info(f"{self._my_name}: agent body step at time {t}")
        stimuli = [s.get_value() for s in self._my_sensors]
        logging.info(f"{self._my_name}: stimuli at time {t} -> {stimuli}")
        self.perceive(stimuli)
        logging.info(f"{self._my_name}: perceptions at time {t} -> {self._my_perceptions}")
        # how about command?

    def get_perceptions(self):
        return self._my_perceptions


class lineFollower(Body):
    _sim: Any
    _cSim_client: Any
    _my_sensors: List
    _my_actuators: List
    _my_perceptions: List
    _obj_handle: Any
    _direction: str
    _stop_sensing: bool
    _robo_name:str

    def __init__(self, name:str, robot_name,lane, perceptions_channel, commands_channel, stop_sensing):
        self.LANE = lane
        self._my_name = name
        self._robo_name = robot_name
        self._cSim_client = RemoteAPIClient()
        self._sim = self._cSim_client.getObject('sim')
        self._my_perc_ch = perceptions_channel
        self._my_comm_ch = commands_channel
        self._stop_sensing = stop_sensing
        self.obstacle_handled = False
        self.parking_handled = True
        self.took_u_turn = False

        if self._robo_name == "/LineTracerEmergency":
            self.parking_handled = False

        self._my_actuators = []
        # Get handles
        print(robot_name)
        self._obj_handle = self._sim.getObject(robot_name)

        motors = [self._sim.getObject(robot_name+"/DynamicLeftJoint"),
                  self._sim.getObject(robot_name+"/DynamicRightJoint")]
        self._my_actuators.append(motors)

        self._my_sensors = []
        self._my_sensors.append( [
            self._sim.getObject(robot_name+"/LeftSensor"), # left color sensor
            self._sim.getObject(robot_name+"/RightSensor"), # right color sensor
            self._sim.getObject(robot_name+"/MiddleSensor"), # middle color sensor
        ])
        self._my_sensors.append([self._sim.getObject(robot_name+"/Proximity_sensor")])

        self._my_msg_broker = redis.Redis(decode_responses=True)
        self._my_perceptions = []
        try:
            self._pub_sub = self._my_msg_broker.pubsub()
            self._pub_sub.subscribe(self._robo_name+"/"+self._my_comm_ch)
            msg = None
            while not msg:
                msg = self._pub_sub.get_message()
                time.sleep(0.1)
                print("Still waiting..")
            print("subscribed: ", msg, "to", self._my_comm_ch)
        except Exception as e:
            logging.exception(e)
            print(e)
            raise e

    def act(self, motor_speeds: List[float]):
        """
        Set the current speed to all motors of the (simulated) robot
        :param motor_speeds: list of values for motors
        :return: True if everything is ok
        """
        try:
            assert len(motor_speeds) == len(self._my_actuators[0])
            for i, speed in enumerate(motor_speeds):
                self._sim.setJointTargetVelocity(self._my_actuators[0][i], motor_speeds[i])
            return True
        except Exception as e:
            print(e)
            logging.exception(e)
            return False

    def _read_color_sensors(self):
        values = []
        for sens in self._my_sensors[0]:
            result = self._sim.readVisionSensor(sens)
            if not (isinstance(result, int)):
                values.append(result[1])
        return values

    def _read_proximity_sensors(self):
        values = []
        for sens in self._my_sensors[1]:
            res, dist, _, _, _ = self._sim.readProximitySensor(sens)
            values.append({"res": res, "dist": dist})
        print(values)
        return values

    def _percept(self, color_sens_values: List, proximity_sense_values: List) -> List[Any]:
        if len(color_sens_values) > 0:
            intensity_left = color_sens_values[0][10]
            intensity_right = color_sens_values[1][10]
            intensity_middle = color_sens_values[2][10]

            red_left = color_sens_values[0][11]
            red_right = color_sens_values[1][11]

            green_left = color_sens_values[0][12]
            green_right = color_sens_values[1][12]

            blue_left = color_sens_values[0][13]
            blue_right = color_sens_values[1][13]

            print('intensity_left', intensity_left)
            print('intensity_right',intensity_right)
            print('intensity_middle', intensity_middle)

            num = 0
            # if num == 7 and self.LANE == 'outer_lane':
            #     print(self.LANE)
            #     return 'lane_change'

            if self._stop_sensing:
                return 'stop'

            if self._robo_name == '/LineTracerEmergency' and not self.obstacle_handled:
                if proximity_sense_values[0]['res'] > 0:
                    print(self.LANE)
                    self.obstacle_handled = True
                    self.took_u_turn = True
                    return 'U-turn'

            elif not self.obstacle_handled:
                if proximity_sense_values[0]['res'] > 0:
                    print(self.LANE)
                    self.obstacle_handled = True
                    return 'lane_change'

            if self.LANE == 'outer_lane':
                if intensity_right >= 0.35 and intensity_right <= 0.45:
                    if red_right == 1.0:
                        return 'stop_on_red'

                    if green_right == 1.0 and not self.parking_handled and self._robo_name == '/LineTracerEmergency':
                        self.parking_handled = True
                        return 'parking_emergency'

                    if blue_right == 1.0 and not self.parking_handled and not self._robo_name == '/LineTracerEmergency':
                        self.parking_handled = True
                        return 'parking'

                if intensity_middle >0.15 and intensity_middle < 0.9:
                    if intensity_middle < 0.45:
                        return 'slight_right'
                    elif intensity_middle > 0.55:
                        return 'slight_left'
                    else:
                        return 'forward'
                if intensity_middle <= 0.15:
                    return 'right'
                if intensity_middle >= 0.9:
                    return 'left'
            else:
                if intensity_left >= 0.35 and intensity_left <= 0.45:
                    if red_left == 1.0:
                        return 'stop_on_red'

                    if green_left == 1.0 and not self.parking_handled and self._robo_name == '/LineTracerEmergency':
                        self.parking_handled = True
                        return 'parking_emergency'

                    if blue_left == 1.0 and not self.parking_handled and not self._robo_name == '/LineTracerEmergency':
                        self.parking_handled = True
                        return 'parking'

                if intensity_middle >0.15 and intensity_middle < 0.9:
                    if intensity_middle < 0.45:
                        return 'slight_left'
                    elif intensity_middle > 0.55:
                        return 'slight_right'
                    else:
                        return 'forward'
                if intensity_middle <= 0.15:
                    return 'left'
                if intensity_middle >= 0.9:
                    return 'right'

    def sense(self):
        """
        Read from (simulated-)hardware sensor devices and store into the internal array
        :return: True if all right, else hardware problem

        readProximitySensor:
        int result,float distance,list detectedPoint,int detectedObjectHandle,list detectedSurfaceNormalVector=sim.readProximitySensor(int sensorHandle)
        """
        try:
            color_sensor_values = self._read_color_sensors()  # only from color sensors
            proximity_sensor_values = self._read_proximity_sensors()
            self._my_perceptions = self._percept(color_sensor_values, proximity_sensor_values)
            return True
        except Exception as e:
            print(e)
            logging.exception(e)
            return False

    def get_right_sensor_intensity(self):
        try:
            values = self._read_color_sensors()
            intensity_right = values[1][10]
            return intensity_right
        except Exception as e:
            print(e)
            logging.exception(e)
            return -1

    def get_left_sensor_intensity(self):
        try:
            values = self._read_color_sensors()
            intensity_right = values[0][10]
            return intensity_right
        except Exception as e:
            print(e)
            logging.exception(e)
            return -1

    def get_left_sensor_green(self):
        try:
            values = self._read_color_sensors()
            intensity_right = values[0][12]
            return intensity_right
        except Exception as e:
            print(e)
            logging.exception(e)
            return -1

    def get_right_sensor_green(self):
        try:
            values = self._read_color_sensors()
            intensity_right = values[1][12]
            return intensity_right
        except Exception as e:
            print(e)
            logging.exception(e)
            return -1

    def get_middle_sensor_intensity(self):
        try:
            values = self._read_color_sensors()
            intensity_right = values[2][10]
            return intensity_right
        except Exception as e:
            print(e)
            logging.exception(e)
            return -1

    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()

    def send_perceptions(self):
        # send the calculated perceptions to the virtual body
        # send the calculated perceptions to the virtual body
        print(self._my_perceptions)
        data_msg = self._my_perceptions
        self._my_msg_broker.publish(self._robo_name+"/"+MY_COMM_CH, data_msg)

    def send_emergency_perceptions(self, message):
        # send the calculated perceptions to the virtual body
        # send the calculated perceptions to the virtual body
        print(message)
        if self._robo_name == '/LineTracer1':
            self._my_msg_broker.publish('/LineTracer2'+"/"+MY_COMM_CH, message)

        if self._robo_name == '/LineTracer2':
            self._my_msg_broker.publish('/LineTracer1'+"/"+MY_COMM_CH, message)

        self._my_msg_broker.publish('/LineTracerEmergency'+"/"+MY_COMM_CH, 'restart_from_parking')
        self._my_msg_broker.publish('/LineTracerEmergency' + "/" + MY_COMM_CH, message)

    def receive_command(self):
        """
        receive the command string from the message broker
        :return:
        """
        msg = self._pub_sub.get_message()
        if msg:
            command = msg['data']
            return command

    def execute(self, command):
        v = 2

        if command == 'stop':
            self.act([0, 0])

        elif command == 'stop_on_red':
            self.act([0, 0])

        elif command == 'parking_emergency' and self.LANE == 'outer_lane':
            if self.took_u_turn:
                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([2, 2])

                while(True):
                    right_sensor = self.get_right_sensor_intensity()
                    if right_sensor > 0.9:
                        break

                time.sleep(1)

                self.act([0, 0])
                while (True):
                    left_sensor = self.get_left_sensor_intensity()
                    green_right = self.get_right_sensor_green()
                    middle_sensor = self.get_middle_sensor_intensity()
                    right_sensor = self.get_right_sensor_intensity()
                    if right_sensor >= 0.35 and right_sensor <= 0.45 and middle_sensor >= 0.35 and middle_sensor <= 0.45 and left_sensor >= 0.35 and left_sensor <= 0.45 and green_right == 1.0:
                        break
                    else:
                        self.act([v + 10, 0])
                        self.act([0, 0])

                while (True):
                    left_sensor = self.get_left_sensor_intensity()
                    middle_sensor = self.get_middle_sensor_intensity()
                    right_sensor = self.get_right_sensor_intensity()
                    if left_sensor > 0.9 and right_sensor > 0.9 and middle_sensor > 0.9:
                        self.act([0, 0])
                        break
                    else:
                        self.act([2, 2])
                self.act([0, 0])

                self._stop_sensing = True
                self.took_u_turn = False
                self.LANE = 'inner_lane'

            else:
                self.obstacle_handled = True

                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])

                proximity_sense_values = self._read_proximity_sensors()
                if proximity_sense_values[0]['res'] > 0:
                    self.act([0, 0])
                    self._stop_sensing = False
                else:
                    self.act([2, 2])
                    self._stop_sensing = True

                time.sleep(3)

                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])

                self.obstacle_handled = True
            self.parking_handled = False

        elif command == 'parking_emergency' and self.LANE == 'inner_lane':
            if self.took_u_turn:
                self.act([2, 2])

                while(True):
                    left_sensor = self.get_left_sensor_intensity()
                    if left_sensor > 0.9:
                        break

                time.sleep(1)

                self.act([0, 0])
                while (True):
                    left_sensor = self.get_left_sensor_intensity()
                    green_left = self.get_left_sensor_green()
                    middle_sensor = self.get_middle_sensor_intensity()
                    right_sensor = self.get_right_sensor_intensity()
                    if right_sensor >= 0.35 and right_sensor <= 0.45 and middle_sensor >= 0.35 and middle_sensor <= 0.45 and left_sensor >= 0.35 and left_sensor <= 0.45 and green_left == 1.0:
                        break
                    else:
                        self.act([0, v + 10])
                        self.act([0, 0])

                while (True):
                    left_sensor = self.get_left_sensor_intensity()
                    middle_sensor = self.get_middle_sensor_intensity()
                    right_sensor = self.get_right_sensor_intensity()
                    if left_sensor > 0.9 and right_sensor > 0.9 and middle_sensor > 0.9:
                        self.act([0, 0])
                        break
                    else:
                        self.act([2, 2])
                self.act([0, 0])

                self._stop_sensing = True
                self.took_u_turn = False
                self.LANE = 'outer_lane'

            else:
                self.obstacle_handled = True

                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])
                self.act([0, v + 10])
                self.act([0, 0])

                proximity_sense_values = self._read_proximity_sensors()
                if proximity_sense_values[0]['res'] > 0:
                    self.act([0, 0])
                    self._stop_sensing = False
                else:
                    self.act([2, 2])
                    self._stop_sensing = True

                time.sleep(3)

                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])
                self.act([v + 10, 0])
                self.act([0, 0])

                self.obstacle_handled = True
            self.parking_handled = False

        elif command == 'parking' and self.LANE == 'inner_lane':

            self.obstacle_handled = True

            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])

            proximity_sense_values = self._read_proximity_sensors()
            if proximity_sense_values[0]['res'] > 0:
                self.act([0, 0])
                self._stop_sensing = False
            else:
                self.act([2, 2])
                self._stop_sensing = True

            time.sleep(3)

            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])

            self.obstacle_handled = True

            self.parking_handled = False

        elif command == 'parking' and self.LANE == 'outer_lane':
            self.obstacle_handled = True
            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])
            self.act([v + 10, 0])
            self.act([0, 0])

            proximity_sense_values = self._read_proximity_sensors()
            if proximity_sense_values[0]['res'] > 0:
                self.act([0, 0])
                self._stop_sensing = False
            else:
                self.act([2, 2])
                self._stop_sensing = True

            time.sleep(3)
            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])
            self.act([0, v + 10])
            self.act([0, 0])

            self.obstacle_handled = False
            self.parking_handled = False

        elif command == 'parking_cmd':
            self.parking_handled = False

        elif command == 'forward':
            self.act([v, v])

        elif command == 'slight_left':
            self.act([v,v+0.3])
            self.act([v, v])

        elif command == 'slight_right':
            self.act([v+0.3, v])
            self.act([v, v])

        elif command == 'left':
            self.act([0,v+10])
            self.act([0, 0])

        elif command == 'right':
            self.act([v+10, 0])
            self.act([0, 0])

        elif (command == 'lane_change' or command == 'emergency_in_inner_lane') and self.LANE == 'outer_lane' and self._robo_name == '/LineTracerEmergency':
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([v + v, v + v])
            while (True):
                if self.get_right_sensor_intensity() > 0.9:
                    self.act([0, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([0, 0])
                    time.sleep(2)
                    break
            self.LANE = 'inner_lane'
            self.obstacle_handled = False

        elif (command == 'lane_change' or command == 'emergency_in_outer_lane') and self.LANE == 'outer_lane' and not self._robo_name =='/LineTracerEmergency':
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([v+v, v+v])
            while(True):
                if self.get_right_sensor_intensity() > 0.9:
                    self.act([0, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([0, 0])
                    time.sleep(2)
                    break
            self.LANE = 'inner_lane'
            self.obstacle_handled = False

        elif (command == 'lane_change' or command == 'emergency_in_outer_lane') and self.LANE == 'inner_lane' and self._robo_name == '/LineTracerEmergency':
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + v, v + v])
            while (True):
                if self.get_left_sensor_intensity() > 0.9:
                    self.act([0, 0])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, 0])
                    time.sleep(2)
                    break
            self.LANE = 'outer_lane'
            self.obstacle_handled = False

        elif (command == 'lane_change' or command == 'emergency_in_inner_lane') and self.LANE == 'inner_lane' and not self._robo_name =='/LineTracerEmergency':
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v+v, v+v])
            while(True):
                if self.get_left_sensor_intensity() > 0.9:
                    self.act([0, 0])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, 0])
                    time.sleep(2)
                    break
            self.LANE = 'outer_lane'
            self.obstacle_handled = False

        elif command =="restart_from_parking"  and self.LANE == 'outer_lane':
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([0, v + 10])
            self.act([v, v])
            self._stop_sensing = False
            while (True):
                if self.get_right_sensor_intensity() < 0.1:
                    self.act([0, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([v + 10, 0])
                    self.act([0, 0])
                    time.sleep(1)
                    break

        elif command =="restart_from_parking" and self.LANE == 'inner_lane':
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v + 10, 0])
            self.act([v, v])
            self._stop_sensing = False
            while (True):
                if self.get_left_sensor_intensity() < 0.1:
                    self.act([0, 0])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, v + 10])
                    self.act([0, 0])
                    time.sleep(1)
                    break

        elif command == "emergency":
            self.act([0, 0])
            self._stop_sensing = True
            msg = ''
            if self.LANE == 'outer_lane':
               msg = 'emergency_in_outer_lane'
            elif self.LANE == 'inner_lane':
                msg = 'emergency_in_inner_lane'
            self.send_emergency_perceptions(msg)

        elif command == "U-turn":
            self.act([0, 0])
            time.sleep(6)
            self.act([-v, -v])
            time.sleep(1)
            if self.LANE == 'outer_lane':
                self.LANE = 'inner_lane'
                self.act([v + 5, 0])
            elif self.LANE == 'inner_lane':
                self.LANE = 'outer_lane'
                self.act([0, v + 5])

            if self.LANE == 'inner_lane':
                while True:
                    if self.get_right_sensor_intensity() < 0.1:
                        self.act([0, 0])
                        time.sleep(2)
                        break

            if self.LANE == 'outer_lane' :
                while True:
                    if self.get_left_sensor_intensity() < 0.1:
                        self.act([0, 0])
                        time.sleep(2)
                        break
            self.act([-v, -v])
            self.obstacle_handled = False

        print(f"command {command} executed by {self._robo_name}")


def run_loop(body):
    i = 0
    while (True):
        try:
            print("********** Loop ", body)
            i = i + 1
            body.sense()
            body.send_perceptions()  # sending through the message broker to the virtual body
            command = body.receive_command()  # from the virtual body
            if command:
                body.execute(command)
        except:
            body.execute('stop')
            body.stop()


if __name__ == "__main__":
    body1 = lineFollower("LineTracerRobo1","/LineTracer1", "outer_lane", MY_PERC_CH, MY_COMM_CH, False)
    body2 = lineFollower("LineTracerRobo2", "/LineTracer2", "outer_lane", MY_PERC_CH, MY_COMM_CH, False)
    bodyEmerg = lineFollower("LineTracerRobo2", "/LineTracerEmergency", "outer_lane", MY_PERC_CH, MY_COMM_CH, True)
    body1.start()
    body2.start()
    bodyEmerg.start()

    t1 = threading.Thread(target=run_loop, args=(body1,))
    t2 = threading.Thread(target=run_loop, args=(body2,))
    t3 = threading.Thread(target=run_loop, args=(bodyEmerg,))

    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()
    # starting thread 2
    t3.start()

    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()
    # wait until thread 2 is completely executed
    t3.join()

    # both threads completely executed
    print("Done!")



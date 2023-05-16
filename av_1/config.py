# Speed
DRIVE_SPEED = 60
NORMAL_SPEED = 60
MAX_SPEED = 100
MIN_SPEED = 40

# Colors
BLACK = 25
WHITE = 60

# Gains
PROPORTIONAL_GAIN = 0.4
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.5



if park != 1 and crash != 1:
    follow lane

    if obstacle:
        switch lane

elif park == 1:
    follow lane


data_stop = False

while True:
    send data
    recieve park status
    recieve emergency status

    #Case 1: Vehicle crashed (client side)
    if crash == 1:
        stop
        change car params to crash params
        while server not ack:
            send emergency
    else:
        #Follow the lane
        follow lane

        # Case 2: If STOP(red) is detected
        if color == "STOP COLOUR":
            stop

        # Case 3: Emergency state
        if emergency == 1:
            parse lane from emergency msg # decide how to be done
            decide whether to switch lane
            change car params to emergency params
            send server ack

        
        # Case 4: Parking (No emergency and no parking)
        elif park == 1:
            # Case 4a: Parking not occupied
            if color == "PARK COLOUR" 
            
                if not obstacle:
                    park 
                    change car params to parking params
                    while server not ack:
                        send parked msg
                        #Best is to remain parked for x time and then restart from this point

                        #or use a flag to determine if it is starting from a parking spot and provide a condition for that
                # Case 4b: Parking occupied
                else:
                    turn and go straight(10cm)

            elif color == "JUNCTION COLOUR":
                go straight(5cm)

        # Case 5: No server or physical interuptions (Just lane following)
        else:
            # Case 5a: Detected color on the lane is parking or junction color
            if color = "PARK COLOUR" or "JUNCTION COLOUR":
                go straight(5cm)

        # Case 6: Obstacle detected
        if obstacle:
            change lane
            change car params for changed lane



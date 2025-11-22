from __future__ import division
import time
import math
import threading
import Adafruit_PCA9685

# --- Hardware Initialization ---
try:
    # busnum=1 is standard for Raspberry Pi
    pwm = Adafruit_PCA9685.PCA9685(busnum=1)
    pwm.set_pwm_freq(50) # Analog servos usually run at 50Hz
except Exception as e:
    print(f"Error initializing PCA9685: {e}")
    exit(1)

# --- Constants & Configuration ---
# Robot Dimensions (mm)
LENGTH_A = 55.0
LENGTH_B = 77.5
LENGTH_C = 27.5
LENGTH_SIDE = 71.0
Z_ABSOLUTE = -28.0

# Movement Constants
Z_DEFAULT = -50.0
Z_UP = -30.0
Z_BOOT = Z_ABSOLUTE
X_DEFAULT = 62.0
X_OFFSET = 0.0
Y_START = 0.0
Y_STEP = 40.0

# Servo Constants
KEEP = 255
PI = math.pi

# Turn Constants calculation
temp_a = math.sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(Y_STEP, 2))
temp_b = 2 * (Y_START + Y_STEP) + LENGTH_SIDE
temp_c = math.sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(2 * Y_START + Y_STEP + LENGTH_SIDE, 2))
temp_alpha = math.acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b)

TURN_X1 = (temp_a - LENGTH_SIDE) / 2
TURN_Y1 = Y_START + Y_STEP / 2
TURN_X0 = TURN_X1 - temp_b * math.cos(temp_alpha)
TURN_Y0 = temp_b * math.sin(temp_alpha) - TURN_Y1 - LENGTH_SIDE

# --- Global State Variables ---
# 4 Legs, 3 Coordinates (x, y, z)
site_now = [[0.0, 0.0, 0.0] for _ in range(4)]
site_expect = [[0.0, 0.0, 0.0] for _ in range(4)]
temp_speed = [[0.0, 0.0, 0.0] for _ in range(4)]

move_speed = 1.0
speed_multiple = 1.0
spot_turn_speed = 4.0
leg_move_speed = 8.0
body_move_speed = 3.0
stand_seat_speed = 1.0

# --- Mapping ---
# Arduino mapped: Leg0(2,3,4), Leg1(5,6,7), Leg2(8,9,10), Leg3(11,12,13)
# PCA9685 map:    Leg0(0,1,2), Leg1(3,4,5), Leg2(6,7,8), Leg3(9,10,11)
LEG_CHANNELS = [
    [0, 1, 2],   # Leg 0
    [3, 4, 5],   # Leg 1
    [6, 7, 8],   # Leg 2
    [9, 10, 11]  # Leg 3
]

# --- Helper Functions ---

def angle_to_pulse(angle):
    """Converts degrees to PCA9685 pulse length (0-4096)."""
    # Standard map: 0deg=150, 180deg=600.
    # You may need to tweak these min/max values for your specific servos.
    pulse = int((angle * 2.5) + 150)
    return max(0, min(4096, pulse))

def cartesian_to_polar(x, y, z):
    """Inverse Kinematics: Calculates alpha, beta, gamma angles."""
    # Calculate w-z degree
    w = (1 if x >= 0 else -1) * (math.sqrt(pow(x, 2) + pow(y, 2)))
    v = w - LENGTH_C
    
    try:
        alpha_rad = math.atan2(z, v) + math.acos((pow(LENGTH_A, 2) - pow(LENGTH_B, 2) + pow(v, 2) + pow(z, 2)) / 2 / LENGTH_A / math.sqrt(pow(v, 2) + pow(z, 2)))
        beta_rad = math.acos((pow(LENGTH_A, 2) + pow(LENGTH_B, 2) - pow(v, 2) - pow(z, 2)) / 2 / LENGTH_A / LENGTH_B)
    except ValueError:
        # Catch math domain errors if target is unreachable
        print("Math Domain Error in IK - Target unreachable")
        return 90, 90, 90

    # Calculate x-y-z degree
    gamma_rad = math.atan2(y, x) if w >= 0 else math.atan2(-y, -x)

    # Convert radians to degrees
    alpha = alpha_rad / PI * 180
    beta = beta_rad / PI * 180
    gamma = gamma_rad / PI * 180
    
    return alpha, beta, gamma

def polar_to_servo(leg, alpha, beta, gamma):
    """Maps kinematic angles to physical servo limits/orientations."""
    if leg == 0:
        alpha = 90 - alpha
        beta = beta
        gamma += 90
    elif leg == 1:
        alpha += 90
        beta = 180 - beta
        gamma = 90 - gamma
    elif leg == 2:
        alpha += 90
        beta = 180 - beta
        gamma = 90 - gamma
    elif leg == 3:
        alpha = 90 - alpha
        beta = beta
        gamma += 90

    pwm.set_pwm(LEG_CHANNELS[leg][0], 0, angle_to_pulse(alpha))
    pwm.set_pwm(LEG_CHANNELS[leg][1], 0, angle_to_pulse(beta))
    pwm.set_pwm(LEG_CHANNELS[leg][2], 0, angle_to_pulse(gamma))

def servo_service_loop():
    """
    Background thread acting as the 'FlexiTimer2' interrupt.
    Runs at ~50Hz to interpolate movement.
    """
    global site_now
    
    while True:
        for i in range(4):
            for j in range(3):
                # Move site_now towards site_expect by temp_speed
                if abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]):
                    site_now[i][j] += temp_speed[i][j]
                else:
                    site_now[i][j] = site_expect[i][j]

            # IK Calculation
            alpha, beta, gamma = cartesian_to_polar(site_now[i][0], site_now[i][1], site_now[i][2])
            polar_to_servo(i, alpha, beta, gamma)
        
        # 20ms delay = 50Hz
        time.sleep(0.02)

def set_site(leg, x, y, z):
    """Sets the target position and calculates speed vector."""
    global site_expect, temp_speed

    length_x = 0
    length_y = 0
    length_z = 0

    if x != KEEP:
        length_x = x - site_now[leg][0]
    if y != KEEP:
        length_y = y - site_now[leg][1]
    if z != KEEP:
        length_z = z - site_now[leg][2]

    length = math.sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2))

    if length == 0:
        length = 0.001 # Avoid div by zero

    temp_speed[leg][0] = length_x / length * move_speed * speed_multiple
    temp_speed[leg][1] = length_y / length * move_speed * speed_multiple
    temp_speed[leg][2] = length_z / length * move_speed * speed_multiple

    if x != KEEP:
        site_expect[leg][0] = x
    if y != KEEP:
        site_expect[leg][1] = y
    if z != KEEP:
        site_expect[leg][2] = z

def wait_reach(leg):
    """Blocking wait until specific leg reaches target."""
    while True:
        if (site_now[leg][0] == site_expect[leg][0] and 
            site_now[leg][1] == site_expect[leg][1] and 
            site_now[leg][2] == site_expect[leg][2]):
            break
        time.sleep(0.001)

def wait_all_reach():
    """Blocking wait until ALL legs reach target."""
    for i in range(4):
        wait_reach(i)

def is_stand():
    return site_now[0][2] == Z_DEFAULT

# --- High Level Movement Functions ---

def sit():
    global move_speed
    move_speed = stand_seat_speed
    for leg in range(4):
        set_site(leg, KEEP, KEEP, Z_BOOT)
    wait_all_reach()

def stand():
    global move_speed
    move_speed = stand_seat_speed
    for leg in range(4):
        set_site(leg, KEEP, KEEP, Z_DEFAULT)
    wait_all_reach()

def turn_left(step):
    global move_speed
    move_speed = spot_turn_speed
    for _ in range(step):
        if site_now[3][1] == Y_START:
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()

            set_site(0, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(1, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(2, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            wait_all_reach()

            set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(2, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(3, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            wait_all_reach()

            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            wait_all_reach()

            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()
        else:
            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()

            set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            set_site(1, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(2, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(3, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(1, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(3, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            wait_all_reach()

            set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            wait_all_reach()

            set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

def turn_right(step):
    global move_speed
    move_speed = spot_turn_speed
    for _ in range(step):
        if site_now[2][1] == Y_START:
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()

            set_site(0, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(1, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            set_site(3, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            wait_all_reach()

            set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(1, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(2, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(3, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            wait_all_reach()

            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            wait_all_reach()

            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()
        else:
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()

            set_site(0, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            set_site(2, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(3, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(0, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(1, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            set_site(2, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            wait_all_reach()

            set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            wait_all_reach()

            set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()

            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

def step_forward(step):
    global move_speed
    move_speed = leg_move_speed
    for _ in range(step):
        if site_now[2][1] == Y_START:
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = body_move_speed

            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = leg_move_speed

            set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()
        else:
            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = body_move_speed

            set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = leg_move_speed

            set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

def step_back(step):
    global move_speed
    move_speed = leg_move_speed
    for _ in range(step):
        if site_now[3][1] == Y_START:
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = body_move_speed

            set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = leg_move_speed

            set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()
        else:
            set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            wait_all_reach()

            move_speed = body_move_speed

            set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
            set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

            move_speed = leg_move_speed

            set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
            wait_all_reach()
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            wait_all_reach()
            set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            wait_all_reach()

def body_left(i):
    set_site(0, site_now[0][0] + i, KEEP, KEEP)
    set_site(1, site_now[1][0] + i, KEEP, KEEP)
    set_site(2, site_now[2][0] - i, KEEP, KEEP)
    set_site(3, site_now[3][0] - i, KEEP, KEEP)
    wait_all_reach()

def body_right(i):
    set_site(0, site_now[0][0] - i, KEEP, KEEP)
    set_site(1, site_now[1][0] - i, KEEP, KEEP)
    set_site(2, site_now[2][0] + i, KEEP, KEEP)
    set_site(3, site_now[3][0] + i, KEEP, KEEP)
    wait_all_reach()

def hand_wave(i):
    global move_speed
    move_speed = 1.0
    if site_now[3][1] == Y_START:
        body_right(15)
        x_tmp = site_now[2][0]
        y_tmp = site_now[2][1]
        z_tmp = site_now[2][2]
        move_speed = body_move_speed
        for _ in range(i):
            set_site(2, TURN_X1, TURN_Y1, 50)
            wait_all_reach()
            set_site(2, TURN_X0, TURN_Y0, 50)
            wait_all_reach()
        set_site(2, x_tmp, y_tmp, z_tmp)
        wait_all_reach()
        move_speed = 1.0
        body_left(15)
    else:
        body_left(15)
        x_tmp = site_now[0][0]
        y_tmp = site_now[0][1]
        z_tmp = site_now[0][2]
        move_speed = body_move_speed
        for _ in range(i):
            set_site(0, TURN_X1, TURN_Y1, 50)
            wait_all_reach()
            set_site(0, TURN_X0, TURN_Y0, 50)
            wait_all_reach()
        set_site(0, x_tmp, y_tmp, z_tmp)
        wait_all_reach()
        move_speed = 1.0
        body_right(15)

def hand_shake(i):
    global move_speed
    move_speed = 1.0
    if site_now[3][1] == Y_START:
        body_right(15)
        x_tmp = site_now[2][0]
        y_tmp = site_now[2][1]
        z_tmp = site_now[2][2]
        move_speed = body_move_speed
        for _ in range(i):
            set_site(2, X_DEFAULT - 30, Y_START + 2 * Y_STEP, 55)
            wait_all_reach()
            set_site(2, X_DEFAULT - 30, Y_START + 2 * Y_STEP, 10)
            wait_all_reach()
        set_site(2, x_tmp, y_tmp, z_tmp)
        wait_all_reach()
        move_speed = 1.0
        body_left(15)
    else:
        body_left(15)
        x_tmp = site_now[0][0]
        y_tmp = site_now[0][1]
        z_tmp = site_now[0][2]
        move_speed = body_move_speed
        for _ in range(i):
            set_site(0, X_DEFAULT - 30, Y_START + 2 * Y_STEP, 55)
            wait_all_reach()
            set_site(0, X_DEFAULT - 30, Y_START + 2 * Y_STEP, 10)
            wait_all_reach()
        set_site(0, x_tmp, y_tmp, z_tmp)
        wait_all_reach()
        move_speed = 1.0
        body_right(15)

def setup():
    print("Robot initialization...")
    # Initialize default parameter
    set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT)
    set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT)
    set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT)
    set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT)
    
    for i in range(4):
        for j in range(3):
            site_now[i][j] = site_expect[i][j]

    # Start Servo Service Thread
    t = threading.Thread(target=servo_service_loop)
    t.daemon = True # Daemon thread exits when main program exits
    t.start()
    
    print("Servo service started.")
    print("Robot initialization Complete.")

if __name__ == "__main__":
    setup()
    try:
        print("Stand")
        stand()
        time.sleep(2)
        
        print("Step forward")
        step_forward(5)
        time.sleep(2)
        
        print("Step back")
        step_back(5)
        time.sleep(2)
        
        print("Turn left")
        turn_left(5)
        time.sleep(2)
        
        print("Turn right")
        turn_right(5)
        time.sleep(2)
        
        print("Hand wave")
        hand_wave(3)
        time.sleep(2)
        
        print("Hand shake")
        hand_shake(3)
        time.sleep(2)
        
        print("Sit")
        sit()
        time.sleep(5)
        
    except KeyboardInterrupt:
        print("Stopped by user.")
        sit()
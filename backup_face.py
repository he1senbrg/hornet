from __future__ import division
import face_recognition
import cv2
import numpy as np
import time
import pickle
import threading
import math
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
LEG_CHANNELS = [
    [0, 1, 2],   # Leg 0
    [3, 4, 5],   # Leg 1
    [6, 7, 8],   # Leg 2
    [9, 10, 11]  # Leg 3
]

# --- Robot Control Helper Functions ---

def angle_to_pulse(angle):
    """Converts degrees to PCA9685 pulse length (0-4096)."""
    pulse = int((angle * 2.5) + 150)
    return max(0, min(4096, pulse))

def cartesian_to_polar(x, y, z):
    """Inverse Kinematics: Calculates alpha, beta, gamma angles."""
    w = (1 if x >= 0 else -1) * (math.sqrt(pow(x, 2) + pow(y, 2)))
    v = w - LENGTH_C
    
    try:
        alpha_rad = math.atan2(z, v) + math.acos((pow(LENGTH_A, 2) - pow(LENGTH_B, 2) + pow(v, 2) + pow(z, 2)) / 2 / LENGTH_A / math.sqrt(pow(v, 2) + pow(z, 2)))
        beta_rad = math.acos((pow(LENGTH_A, 2) + pow(LENGTH_B, 2) - pow(v, 2) - pow(z, 2)) / 2 / LENGTH_A / LENGTH_B)
    except ValueError:
        print("Math Domain Error in IK - Target unreachable")
        return 90, 90, 90

    gamma_rad = math.atan2(y, x) if w >= 0 else math.atan2(-y, -x)

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
    """Background thread for servo interpolation at ~50Hz."""
    global site_now
    
    while True:
        for i in range(4):
            for j in range(3):
                if abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]):
                    site_now[i][j] += temp_speed[i][j]
                else:
                    site_now[i][j] = site_expect[i][j]

            alpha, beta, gamma = cartesian_to_polar(site_now[i][0], site_now[i][1], site_now[i][2])
            polar_to_servo(i, alpha, beta, gamma)
        
        time.sleep(0.02)

def set_site(leg, x, y, z):
    """Sets the target position and calculates speed vector."""
    global site_expect, temp_speed

    length_x = 0 if x == KEEP else x - site_now[leg][0]
    length_y = 0 if y == KEEP else y - site_now[leg][1]
    length_z = 0 if z == KEEP else z - site_now[leg][2]

    length = math.sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2))
    if length == 0:
        length = 0.001

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

def robot_setup():
    print("Robot initialization...")
    set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT)
    set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT)
    set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT)
    set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_BOOT)
    
    for i in range(4):
        for j in range(3):
            site_now[i][j] = site_expect[i][j]

    # Start Servo Service Thread
    t = threading.Thread(target=servo_service_loop)
    t.daemon = True
    t.start()
    
    print("Servo service started.")
    print("Robot initialization Complete.")

# Load pre-trained face encodings
print("[INFO] loading encodings...")
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]

# Initialize the USB webcam (0 is usually the default camera index)
cap = cv2.VideoCapture(0)

# Set resolution (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize our variables
cv_scaler = 4  # this has to be a whole number

face_locations = []
face_encodings = []
face_names = []
frame_count = 0
start_time = time.time()
fps = 0

# Gesture control variables
gesture_lock = threading.Lock()
gesture_in_progress = False
last_greeted_person = None
greeting_cooldown = 10  # seconds before greeting same person again
last_greeting_time = 0

def process_frame(frame):
    global face_locations, face_encodings, face_names
    
    # Resize the frame using cv_scaler to increase performance
    resized_frame = cv2.resize(frame, (0, 0), fx=(1/cv_scaler), fy=(1/cv_scaler))
    
    # Convert the image from BGR to RGB colour space
    rgb_resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
    
    # Find all the faces and face encodings in the current frame of video
    face_locations = face_recognition.face_locations(rgb_resized_frame)
    face_encodings = face_recognition.face_encodings(rgb_resized_frame, face_locations, model='large')
    
    face_names = []
    for face_encoding in face_encodings:
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"
        
        # Use the known face with the smallest distance to the new face
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
        face_names.append(name)
    
    return frame

def draw_results(frame):
    # Display the results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled
        top *= cv_scaler
        right *= cv_scaler
        bottom *= cv_scaler
        left *= cv_scaler
        
        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 255), 2)
        
        # Draw a label with a name below the face
        cv2.rectangle(frame, (left - 3, top - 35), (right + 3, top), (0, 255, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, top - 6), font, 1.0, (0, 0, 0), 2)
        
    return frame

def calculate_fps():
    global frame_count, start_time, fps
    frame_count += 1
    elapsed_time = time.time() - start_time
    if elapsed_time > 1:
        fps = frame_count / elapsed_time
        frame_count = 0
        start_time = time.time()
    return fps

def perform_gesture(name):
    """Execute robot gesture in a separate thread"""
    global gesture_in_progress, last_greeted_person, last_greeting_time
    
    with gesture_lock:
        if gesture_in_progress:
            return
        gesture_in_progress = True
    
    try:
        print(f"[GESTURE] Greeting {name}...")
        
        # Specific gestures for specific people
        if name in ["Souri", "Vishnu", "Arjun"]:
            hand_wave(3)  # Wave 3 times for these specific people
            print(f"[GESTURE] Waved at {name}")
            hand_shake(3)  # Shake 3 times
            print(f"[GESTURE] Handshake for {name}")
            step_forward(6)  # Step forward once
            print(f"[GESTURE] Step forward for {name}")
            step_back(6)  # Step back to original position
            print(f"[GESTURE] Step back for {name}")
        
        last_greeted_person = name
        last_greeting_time = time.time()
        
    except Exception as e:
        print(f"[ERROR] Gesture failed: {e}")
    finally:
        with gesture_lock:
            gesture_in_progress = False

def check_and_greet():
    """Check if we should greet any detected known faces"""
    global last_greeted_person, last_greeting_time, gesture_in_progress
    
    # Don't trigger if gesture is already in progress
    if gesture_in_progress:
        return
    
    # Check for known faces that should receive gestures
    # Only greet Souri, Vishnu, or Arjun
    allowed_names = ["Souri", "Vishnu", "Arjun"]
    known_faces = [name for name in face_names if name in allowed_names]
    
    if not known_faces:
        return
    
    # Get the first known face detected
    person_to_greet = known_faces[0]
    current_time = time.time()
    
    # Check cooldown - don't greet same person too frequently
    if (last_greeted_person == person_to_greet and 
        (current_time - last_greeting_time) < greeting_cooldown):
        return
    
    # If it's a new person or cooldown expired, start greeting
    if (last_greeted_person != person_to_greet or 
        (current_time - last_greeting_time) >= greeting_cooldown):
        
        # Start gesture in separate thread so it doesn't block video
        gesture_thread = threading.Thread(target=perform_gesture, args=(person_to_greet,))
        gesture_thread.daemon = True
        gesture_thread.start()

# Initialize robot
print("[ROBOT] Initializing robot...")
robot_setup()
time.sleep(1)
print("[ROBOT] Standing up...")
stand()
time.sleep(1)
print("[ROBOT] Ready!")

try:
    while True:
        # Capture a frame from camera
        ret, frame = cap.read()
        
        if not ret:
            print("[ERROR] Failed to capture frame")
            break
        
        # Process the frame with the function
        processed_frame = process_frame(frame)
        
        # Check if we should greet anyone (non-blocking)
        check_and_greet()
        
        # Get the text and boxes to be drawn based on the processed frame
        display_frame = draw_results(processed_frame)
        
        # Calculate and update FPS
        current_fps = calculate_fps()
        
        # Attach FPS counter to the text and boxes
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}", (display_frame.shape[1] - 150, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Show gesture status
        if gesture_in_progress:
            cv2.putText(display_frame, "GESTURING...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        # Display everything over the video feed
        cv2.imshow('Face Rec Running', display_frame)
        
        # Break the loop and stop the script if 'q' is pressed
        if cv2.waitKey(1) == ord("q"):
            break

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user")

finally:
    # Cleanup
    print("[ROBOT] Sitting down...")
    sit()
    time.sleep(1)
    
    print("[INFO] Releasing resources...")
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Shutdown complete")
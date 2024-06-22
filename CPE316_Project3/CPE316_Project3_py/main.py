import cv2                                  # Import OpenCV Library for Image Detection
import serial                               # Import pyserial Library for UART Communication with MCU
import threading                            # Used to Create Separate Threads for Reading and Writing Simultaneously
import time                                 # Used for Delays
import numpy as np                          # Array Manipulation Used for Kalman Filter

# MACROS #
SERIAL_PORT = 'COM5'                        # STM32 Port
BAUD_RATE = 115200                          # UART Baud Rate
CONSECUTIVE_MISSING_THRESHOLD = 10           # Number of consecutive frames to consider the face as lost
UART_INIT_DELAY = 1                         # Delay for UART Connection (seconds)
FLIP_ORIENTATION = 1                        # Horizontal Flip
SCALE_FACTOR = 1.3                          # Scales Frame Size of Detection
MIN_NEIGHBORS = 7                           # Adjusts Certainty of Detector
WHITE_PIXEL = (255, 255, 255)               # Color Selector
BLUE_PIXEL = (255, 0, 0)                    # Color Selector
GREEN_PIXEL = (0, 255, 0)                   # Color Selector
DEF_SERVO_VAL = ['250', '250']              # Default State of Servo
SERVOX_CAL = (50, 500, 580, 800)            # Calibration for ServoX
SERVOY_CAL = (50, 500, 865, 625)            # Calibration for ServoY
SERVOX_ITER = 0                             # Iteration Index for ServoX
SERVOY_ITER = 1                             # Iteration Index for ServoY
SERIAL_PORT_DELAY = 0.025                   # Used to Prevent Overflooding of Data

# Global Variables #
consecutive_missing_frames = 0              # Used to Prevent Accidental Misses
face_detected = False                       # Is True if no Continuous Misses
Servo_iter = 0                              # Used for Handling ServoX and ServoY
Exit_Flag = threading.Event()               # Used to Safely Exit Program

# Peripheral Initialization #
# Configure the Serial Port to Terminal #
port = serial.Serial(
    port=SERIAL_PORT,  # Share COM5 with MCU
    baudrate=BAUD_RATE,  # Share Baud Rate with MCU
    bytesize=serial.EIGHTBITS,  # 8 Data Bits
    parity=serial.PARITY_NONE,  # No Parity Bit
    stopbits=serial.STOPBITS_ONE)  # One Stop Bit
# Wait for Communication to be Established #
time.sleep(UART_INIT_DELAY)

# Initialize Webcam Peripheral #
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)
# Load Facial and Eye Classifier #
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
profile_face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')

# Initialize Kalman Filter #
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.eye(2, 4, dtype=np.float32)
kalman.transitionMatrix = np.eye(4, 4, dtype=np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
kalman.statePre = np.zeros((4, 1), dtype=np.float32)
kalman.statePost = np.zeros((4, 1), dtype=np.float32)
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-4


# Functions #
# Scale the Value from the Original Range to the New Range #
def map_value(value, from_min, from_max, to_min, to_max):
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

# Detect faces using the specified cascade and grayscale frame
def detect_face(cascade, grey):
    return cascade.detectMultiScale(grey, scaleFactor=SCALE_FACTOR, minNeighbors=MIN_NEIGHBORS)

# Process the detected face, update Kalman filter, and map coordinates
def process_detection(frame, faces):
    global consecutive_missing_frames, face_detected
    for (x, y, w, h) in faces:
        consecutive_missing_frames = 0
        face_detected = True
        # Using Facial Detection Frame, Create Box Around Face #
        cv2.rectangle(frame, (x, y), (x + w, y + h), WHITE_PIXEL, 1)
        # Find Center Point of Face #
        center_x = (x + w // 2)
        center_y = (y + h // 2)
        # Update Kalman Filter
        measurement = np.array([[np.float32(center_x)], [np.float32(center_y)]], np.float32)
        kalman.correct(measurement)
        # Scale Coordinate to Servo Range #
        center_x_conv = int(map_value(center_x, *SERVOX_CAL))
        center_y_conv = int(map_value(center_y, *SERVOY_CAL))
        print(f"Target Point: ({center_x_conv}, {center_y_conv})")
        cv2.circle(frame, (center_x, center_y), 2, BLUE_PIXEL, -1)
        # Send Coordinates to Terminal to be Received by MCU
        return [str(center_x_conv), str(center_y_conv)], frame
    return None, frame

# Read Frame, Apply Filter for Algorithm, and Retrieve Points of Match #
def image_data():
    global consecutive_missing_frames, face_detected
    # Read Frame and Flip Horizontally to Make Calibration Easier #
    ret, frame = cap.read()
    frame = cv2.flip(frame, FLIP_ORIENTATION)
    # Apply Grayscale Filter #
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Retrieve Facial Detection #
    faces = detect_face(face_cascade, grey)

    # If Face is Detected #
    if len(faces) > 0:
        return process_detection(frame, faces)

    # Second Attempt: Detect Face Again with a Different Dataset #
    faces = detect_face(profile_face_cascade, grey)
    if len(faces) > 0:
        return process_detection(frame, faces)

    # Face Cannot Be Detected; Either Predict Next Coordinate or Assume No Face is Present #
    consecutive_missing_frames += 1
    # If Multiple Misses in a Row, Assume No Face is Present #
    if consecutive_missing_frames > CONSECUTIVE_MISSING_THRESHOLD:
        face_detected = False
        return DEF_SERVO_VAL, frame

    # To Alleviate Distortion, Use Kalman Filter to Predict Position #
    prediction = kalman.predict()
    pred_x, pred_y = int(prediction[0]), int(prediction[1])
    print(f"Predicted Point: ({pred_x}, {pred_y})")
    cv2.circle(frame, (pred_x, pred_y), 2, GREEN_PIXEL, -1)
    # Scale Predicted Coordinate to Servo Range #
    pred_x_conv = int(map_value(pred_x, *SERVOX_CAL))
    pred_y_conv = int(map_value(pred_y, *SERVOY_CAL))
    return [str(pred_x_conv), str(pred_y_conv)], frame

# Read and Display Data from MCU onto Terminal
# Executes in Separate Thread Continuously #
def read_MCU():
    while not Exit_Flag.is_set():
        if port.in_waiting > 0:
            data = port.readline().decode('utf-8').rstrip()
            if data:
                print(f"STM32: {data}")

# Write Servo Coordinate Values to MCU
# Executes in Separate Thread Continuously #
def write_MCU():
    global Servo_iter
    while not Exit_Flag.is_set():
        # Gather Frame Every Other Iteration to
        # Transmit Both ServoX and ServoY Values #
        dataArr, frame = image_data()

        # Display Frame #
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            Exit_Flag.set()
            break

        # Servo_iter Duplexes ServoX and ServoY Input #
        if Servo_iter == SERVOX_ITER:
            user_input = f"X{dataArr[0]}\n"
            Servo_iter = SERVOY_ITER
        else:
            user_input = f"Y{dataArr[1]}\n"
            Servo_iter = SERVOX_ITER

        # Transmit Data to MCU #
        port.write(user_input.encode('utf-8'))
        # Ensure all Data is Sent Immediately #
        port.flush()
        # Delay to Avoid Flooding the Serial Port #
        time.sleep(SERIAL_PORT_DELAY)

# Main Program #
# Create Two Separate Threads to Allow for Simultaneous Reception and Transmission with MCU #
def main():
    # Create Reception and Transmission Thread #
    read_thread = threading.Thread(target=read_MCU)
    write_thread = threading.Thread(target=write_MCU)

    # Activate Both Threads to Run Independently #
    read_thread.start()
    write_thread.start()

    # Close Threads when Ready #
    read_thread.join()
    write_thread.join()

# main.py #
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Set Exit Flag #
        Exit_Flag.set()
        # Reset Everything #
        cap.release()
        cv2.destroyAllWindows()

        # Return Laser to Default Position #
        port.write("X250\n".encode('utf-8'))
        port.flush()
        time.sleep(SERIAL_PORT_DELAY)
        port.write("Y250\n".encode('utf-8'))
        port.flush()
        time.sleep(SERIAL_PORT_DELAY)


# Facial Expression Control of LCD Display - Python Script
# This script captures webcam video, detects facial expressions using MediaPipe,
# and sends commands to Arduino to control an LCD display

import cv2
import mediapipe as mp
import serial
import math
import time

class FacialExpressionDetector:
    def __init__(self, webcam_id=1, serial_port='COM5', baud_rate=9600):
        """
        Initialize the facial expression detector
        
        Args:
            webcam_id: ID of the webcam (0 or 1 typically)
            serial_port: Arduino's serial port (e.g., 'COM5' on Windows, '/dev/ttyUSB0' on Linux)
            baud_rate: Serial communication speed (must match Arduino)
        """
        # Initialize MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Initialize MediaPipe drawing utilities for visualization
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize webcam
        self.webcam = cv2.VideoCapture(webcam_id)
        if not self.webcam.isOpened():
            print(f"Error: Could not open webcam with ID {webcam_id}")
            print("Trying webcam ID 0...")
            self.webcam = cv2.VideoCapture(0)
            if not self.webcam.isOpened():
                raise Exception("Could not open any webcam")
        
        # Initialize serial communication with Arduino
        try:
            self.arduino = serial.Serial(serial_port, baud_rate)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to Arduino on {serial_port}")
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")
            print("Please check the serial port and try again")
            raise
        
        # Landmark indices for lip corners (MediaPipe face mesh)
        self.LEFT_LIP_CORNER = 61   # Left corner of lips
        self.RIGHT_LIP_CORNER = 306 # Right corner of lips
        
        print("Facial Expression Detector initialized successfully!")
        print("Press ESC to exit")

    def calculate_lip_distance(self, landmarks, frame_width, frame_height):
        """
        Calculate the distance between lip corners to determine expression
        
        Args:
            landmarks: MediaPipe face landmarks
            frame_width: Width of the video frame
            frame_height: Height of the video frame
            
        Returns:
            tuple: (distance, point1_coords, point2_coords)
        """
        # Get coordinates of lip corner landmarks
        point1 = landmarks.landmark[self.LEFT_LIP_CORNER]
        point2 = landmarks.landmark[self.RIGHT_LIP_CORNER]
        
        # Convert normalized coordinates to pixel coordinates
        x1 = int(point1.x * frame_width)
        y1 = int(point1.y * frame_height)
        x2 = int(point2.x * frame_width)
        y2 = int(point2.y * frame_height)
        
        # Calculate Euclidean distance between the two points
        distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
        
        return distance, (x1, y1), (x2, y2)

    def send_expression_to_arduino(self, expression):
        """
        Send expression command to Arduino via serial communication
        
        Args:
            expression: String indicating the expression ('smiling' or 'sad')
        """
        try:
            if expression == 'smiling':
                self.arduino.write(b'A')  # Send 'A' for smiling
                print("Sent: Smiling")
            elif expression == 'sad':
                self.arduino.write(b'B')  # Send 'B' for sad
                print("Sent: Sad")
        except serial.SerialException as e:
            print(f"Error sending data to Arduino: {e}")

    def detect_expression(self, distance):
        """
        Determine facial expression based on lip corner distance
        
        Args:
            distance: Distance between lip corners
            
        Returns:
            String: 'smiling', 'sad', or 'normal'
        """
        if distance > 48:
            return 'smiling'
        elif distance < 46:
            return 'sad'
        else:
            return 'normal'  # Neutral expression

    def draw_landmarks(self, frame, point1, point2):
        """
        Draw circles at landmark points for visualization
        
        Args:
            frame: Video frame to draw on
            point1: First landmark coordinates (x1, y1)
            point2: Second landmark coordinates (x2, y2)
        """
        # Draw circles at lip corner landmarks
        cv2.circle(frame, point1, 5, (0, 255, 0), -1)  # Green circle
        cv2.circle(frame, point2, 5, (0, 255, 0), -1)  # Green circle
        
        # Draw line connecting the points
        cv2.line(frame, point1, point2, (255, 0, 0), 2)  # Blue line

    def run(self):
        """
        Main loop for facial expression detection
        """
        last_expression = None  # Track last sent expression to avoid spam
        
        while True:
            # Read frame from webcam
            ret, frame = self.webcam.read()
            if not ret:
                print("Error: Could not read frame from webcam")
                break
            
            # Get frame dimensions
            height, width, _ = frame.shape
            
            # Convert BGR to RGB (MediaPipe uses RGB)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe Face Mesh
            results = self.face_mesh.process(rgb_frame)
            
            # Check if faces are detected
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    # Calculate distance between lip corners
                    distance, point1, point2 = self.calculate_lip_distance(
                        face_landmarks, width, height
                    )
                    
                    # Draw landmarks on frame for visualization
                    self.draw_landmarks(frame, point1, point2)
                    
                    # Detect expression based on distance
                    current_expression = self.detect_expression(distance)
                    
                    # Display distance and expression on frame
                    cv2.putText(frame, f'Distance: {distance:.2f}', 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(frame, f'Expression: {current_expression}', 
                              (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    # Send command to Arduino only if expression changed
                    if current_expression != last_expression and current_expression != 'normal':
                        self.send_expression_to_arduino(current_expression)
                        last_expression = current_expression
            
            # Display the frame
            cv2.imshow('Facial Expression Detector', frame)
            
            # Check for ESC key press to exit
            key = cv2.waitKey(10) & 0xFF
            if key == 27:  # ESC key
                break
        
        # Cleanup
        self.cleanup()

    def cleanup(self):
        """
        Clean up resources
        """
        print("Cleaning up...")
        self.webcam.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'arduino'):
            self.arduino.close()
        print("Cleanup complete")

def main():
    """
    Main function to run the facial expression detector
    """
    try:
        # Initialize detector (adjust webcam_id and serial_port as needed)
        detector = FacialExpressionDetector(
            webcam_id=0,          # 0 for laptop camera, 1 for external camera
            serial_port='COM5',   # Change to your Arduino's port (COM3, COM4, etc.)
            baud_rate=9600
        )
        
        # Start detection
        detector.run()
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Program ended")

if __name__ == "__main__":
    main()
# Facial Expression Detection - No Arduino Version
# This version works with just your laptop camera and displays results on screen

import cv2
import mediapipe as mp
import math
import time

class FacialExpressionDetector:
    def __init__(self, webcam_id=0):
        """
        Initialize the facial expression detector (laptop camera only)
        
        Args:
            webcam_id: ID of the webcam (0 for laptop camera, 1 for external)
        """
        print("🎯 Initializing Facial Expression Detector (No Arduino Mode)")
        
        # Initialize MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Initialize MediaPipe drawing utilities for visualization
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize webcam (try common IDs)
        self.webcam = cv2.VideoCapture(webcam_id)
        if not self.webcam.isOpened():
            print(f"❌ Error: Could not open webcam with ID {webcam_id}")
            if webcam_id != 0:
                print("🔄 Trying laptop camera (ID 0)...")
                self.webcam = cv2.VideoCapture(0)
            else:
                print("🔄 Trying external camera (ID 1)...")
                self.webcam = cv2.VideoCapture(1)
            if not self.webcam.isOpened():
                raise Exception("❌ Could not open any webcam")
        
        # Set camera resolution for better performance
        self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Landmark indices for lip corners (MediaPipe face mesh)
        self.LEFT_LIP_CORNER = 61   # Left corner of lips
        self.RIGHT_LIP_CORNER = 306 # Right corner of lips
        
        # Expression tracking
        self.current_expression = "Normal"
        self.expression_history = []
        self.last_expression_time = 0
        
        print("✅ Facial Expression Detector initialized successfully!")
        print("📹 Camera opened successfully")
        print("🎯 Ready to detect expressions!")
        print("\n📋 Instructions:")
        print("   😊 SMILE WIDELY to trigger 'SMILING' detection")
        print("   😢 FROWN or make sad face to trigger 'SAD' detection")
        print("   😐 Keep neutral face for 'NORMAL'")
        print("   ⌨️  Press ESC to exit")
        print("=" * 60)

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

    def detect_expression(self, distance):
        """
        Determine facial expression based on lip corner distance
        
        Args:
            distance: Distance between lip corners
            
        Returns:
            String: 'SMILING', 'SAD', or 'NORMAL'
        """
        if distance > 48:
            return 'SMILING'
        elif distance < 46:
            return 'SAD'
        else:
            return 'NORMAL'

    def smooth_expression(self, new_expression):
        """
        Smooth expression detection to avoid flickering
        
        Args:
            new_expression: Newly detected expression
            
        Returns:
            String: Smoothed expression
        """
        # Add to history
        self.expression_history.append(new_expression)
        
        # Keep only last 5 detections
        if len(self.expression_history) > 5:
            self.expression_history.pop(0)
        
        # Count occurrences
        expression_counts = {}
        for expr in self.expression_history:
            expression_counts[expr] = expression_counts.get(expr, 0) + 1
        
        # Return most common expression
        return max(expression_counts.items(), key=lambda x: x[1])[0]

    def simulate_arduino_display(self, expression):
        """
        Simulate what would be displayed on Arduino LCD
        
        Args:
            expression: Current expression to display
        """
        current_time = time.time()
        
        # Only update if expression changed or every 2 seconds
        if (expression != self.current_expression or 
            current_time - self.last_expression_time > 2):
            
            self.current_expression = expression
            self.last_expression_time = current_time
            
            # Simulate LCD display
            lcd_display = "=" * 16 + "\n"
            if expression == 'SMILING':
                lcd_display += "|   SMILING :)   |\n"
            elif expression == 'SAD':
                lcd_display += "|    SAD :(      |\n"
            else:
                lcd_display += "|    NORMAL      |\n"
            lcd_display += "=" * 16
            
            print(f"\n🖥️  LCD Display Simulation:")
            print(lcd_display)
            
            # Would send to Arduino here:
            if expression == 'SMILING':
                print("📡 Would send: 'A' to Arduino (SMILING)")
            elif expression == 'SAD':
                print("📡 Would send: 'B' to Arduino (SAD)")

    def draw_interface(self, frame, distance, expression, point1, point2):
        """
        Draw user interface elements on the frame
        
        Args:
            frame: Video frame to draw on
            distance: Current lip distance
            expression: Current expression
            point1, point2: Landmark coordinates
        """
        height, width = frame.shape[:2]
        
        # Draw title bar
        cv2.rectangle(frame, (0, 0), (width, 80), (50, 50, 50), -1)
        cv2.putText(frame, 'Facial Expression Detector - No Arduino Mode', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, 'Press ESC to exit', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Draw landmarks (lip corners)
        cv2.circle(frame, point1, 6, (0, 255, 0), -1)  # Green circle
        cv2.circle(frame, point2, 6, (0, 255, 0), -1)  # Green circle
        cv2.line(frame, point1, point2, (255, 0, 0), 3)  # Blue line
        
        # Create info panel
        panel_height = 120
        cv2.rectangle(frame, (0, height - panel_height), (width, height), (40, 40, 40), -1)
        
        # Display measurements and expression
        y_offset = height - panel_height + 25
        
        cv2.putText(frame, f'Lip Distance: {distance:.1f}px', 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f'Expression: {expression}', 
                   (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Expression thresholds
        cv2.putText(frame, f'Thresholds: >48=SMILE, <46=SAD', 
                   (10, y_offset + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Arduino simulation status
        cv2.putText(frame, f'Arduino: SIMULATED', 
                   (10, y_offset + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 1)
        
        # Draw expression indicator
        indicator_color = (0, 255, 0) if expression == 'SMILING' else (0, 0, 255) if expression == 'SAD' else (128, 128, 128)
        cv2.rectangle(frame, (width - 100, height - panel_height + 10), (width - 10, height - panel_height + 50), indicator_color, -1)
        cv2.putText(frame, expression[:4], (width - 90, height - panel_height + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def run(self):
        """
        Main loop for facial expression detection
        """
        print("\n🚀 Starting facial expression detection...")
        print("📸 Look at your camera and make different expressions!")
        
        frame_count = 0
        
        while True:
            # Read frame from webcam
            ret, frame = self.webcam.read()
            if not ret:
                print("❌ Error: Could not read frame from webcam")
                break
            
            frame_count += 1
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Get frame dimensions
            height, width, _ = frame.shape
            
            # Convert BGR to RGB (MediaPipe uses RGB)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe Face Mesh
            results = self.face_mesh.process(rgb_frame)
            
            # Default values
            distance = 0
            expression = "NO FACE"
            point1, point2 = (0, 0), (0, 0)
            
            # Check if faces are detected
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    # Calculate distance between lip corners
                    distance, point1, point2 = self.calculate_lip_distance(
                        face_landmarks, width, height
                    )
                    
                    # Detect expression based on distance
                    raw_expression = self.detect_expression(distance)
                    expression = self.smooth_expression(raw_expression)
                    
                    # Simulate Arduino display every 30 frames (about 1 second at 30fps)
                    if frame_count % 30 == 0:
                        self.simulate_arduino_display(expression)
                    
                    break  # Only process first detected face
            
            # Draw interface elements
            self.draw_interface(frame, distance, expression, point1, point2)
            
            # Display the frame
            cv2.imshow('Facial Expression Detector - No Arduino Mode', frame)
            
            # Check for ESC key press to exit
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                break
        
        # Cleanup
        self.cleanup()

    def cleanup(self):
        """
        Clean up resources
        """
        print("\n🧹 Cleaning up...")
        self.webcam.release()
        cv2.destroyAllWindows()
        print("✅ Cleanup complete")
        print("👋 Thank you for using Facial Expression Detector!")

def main():
    """
    Main function to run the facial expression detector
    """
    print("🎮 Facial Expression Detection - No Arduino Version")
    print("=" * 60)
    
    try:
        # Initialize detector (try different camera IDs if needed)
        detector = FacialExpressionDetector(webcam_id=0)  # 0 for laptop camera
        
        # Start detection
        detector.run()
        
    except KeyboardInterrupt:
        print("\n⏹️  Program interrupted by user")
    except Exception as e:
        print(f"❌ An error occurred: {e}")
        print("\n💡 Troubleshooting tips:")
        print("   1. Make sure your camera is not being used by another application")
        print("   2. Try changing webcam_id to 1 in the main() function")
        print("   3. Check if camera permissions are enabled")
    finally:
        print("🏁 Program ended")

if __name__ == "__main__":
    main()
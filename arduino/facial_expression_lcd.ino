/*
  Facial Expression LCD Controller - Arduino Sketch
  
  This sketch receives serial commands from a Python script and displays
  corresponding facial expressions on a 16x2 I2C LCD display.
  
  Hardware Connections:
  - LCD GND  -> Arduino GND
  - LCD VCC  -> Arduino 5V
  - LCD SDA  -> Arduino A4 (Analog 4)
  - LCD SCL  -> Arduino A5 (Analog 5)
  
  Commands:
  - 'A' = Display "smiling"
  - 'B' = Display "sad"
*/

#include <LiquidCrystal_I2C.h>

// Initialize LCD with I2C address 0x27, 16 columns, 2 rows
// Note: If 0x27 doesn't work, try 0x3F or run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variable to store incoming serial data
char incomingData;

// Variables to track current state and prevent unnecessary updates
String currentExpression = "";
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_DELAY = 100; // Minimum delay between updates (ms)

void setup() {
  /*
    Initialize LCD and serial communication
  */
  
  // Initialize the LCD display
  lcd.init();
  
  // Turn on the backlight
  lcd.backlight();
  
  // Initialize serial communication with computer (Python script)
  // Baud rate must match the Python script (9600)
  Serial.begin(9600);
  
  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("Facial Expression");
  lcd.setCursor(0, 1);
  lcd.print("Detector Ready");
  
  // Wait for 2 seconds to show startup message
  delay(2000);
  
  // Clear the screen and set initial state
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting...");
  
  // Send ready signal to Python (optional)
  Serial.println("Arduino Ready");
}

void loop() {
  /*
    Main loop: Check for incoming serial data and update LCD accordingly
  */
  
  // Check if data is available from Python script
  if (Serial.available() > 0) {
    // Read the incoming byte
    incomingData = Serial.read();
    
    // Process the received command
    processCommand(incomingData);
  }
  
  // Small delay to prevent overwhelming the serial buffer
  delay(10);
}

void processCommand(char command) {
  /*
    Process the received command and update LCD display
    
    Parameters:
    - command: Character received from Python ('A' or 'B')
  */
  
  // Check if enough time has passed since last update to prevent flickering
  if (millis() - lastUpdateTime < UPDATE_DELAY) {
    return;
  }
  
  String newExpression = "";
  
  // Determine the expression based on received command
  switch (command) {
    case 'A':
      newExpression = "smiling";
      break;
      
    case 'B':
      newExpression = "sad";
      break;
      
    default:
      // Unknown command received
      Serial.print("Unknown command: ");
      Serial.println(command);
      return;
  }
  
  // Only update display if expression has changed
  if (newExpression != currentExpression) {
    updateDisplay(newExpression);
    currentExpression = newExpression;
    lastUpdateTime = millis();
    
    // Send confirmation back to Python (optional, for debugging)
    Serial.print("Displayed: ");
    Serial.println(newExpression);
  }
}

void updateDisplay(String expression) {
  /*
    Update the LCD display with the new expression
    
    Parameters:
    - expression: String to display on LCD ("smiling" or "sad")
  */
  
  // Clear the entire LCD screen
  lcd.clear();
  
  // Set cursor to beginning of first line
  lcd.setCursor(0, 0);
  
  // Print the main expression
  lcd.print("Expression:");
  
  // Set cursor to beginning of second line
  lcd.setCursor(0, 1);
  
  // Print the detected expression with formatting
  if (expression == "smiling") {
    lcd.print("   SMILING :)");
  } else if (expression == "sad") {
    lcd.print("    SAD :(");
  } else {
    lcd.print("   " + expression);
  }
}

void displayError(String errorMsg) {
  /*
    Display error message on LCD (utility function)
    
    Parameters:
    - errorMsg: Error message to display
  */
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Error:");
  lcd.setCursor(0, 1);
  lcd.print(errorMsg);
  
  Serial.print("Error: ");
  Serial.println(errorMsg);
}

void displayStatus(String statusMsg) {
  /*
    Display status message on LCD (utility function)
    
    Parameters:
    - statusMsg: Status message to display
  */
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Status:");
  lcd.setCursor(0, 1);
  lcd.print(statusMsg);
  
  Serial.print("Status: ");
  Serial.println(statusMsg);
}

/*
  Additional Notes:
  
  1. I2C Address: The most common I2C addresses for LCD displays are:
     - 0x27 (most common)
     - 0x3F
     - 0x20
     If the display doesn't work, try changing the address in the
     LiquidCrystal_I2C constructor.
  
  2. Troubleshooting I2C:
     - Use an I2C scanner sketch to find the correct address
     - Check wiring connections (SDA to A4, SCL to A5 on Uno)
     - Ensure proper power supply (5V and GND)
  
  3. Serial Communication:
     - Baud rate must match Python script (9600)
     - Ensure correct COM port is selected in Arduino IDE
     - Use Serial Monitor for debugging
  
  4. Power Requirements:
     - LCD typically requires 5V
     - Ensure Arduino can provide sufficient current
     - Consider external power supply for larger projects
  
  5. Customization:
     - Modify display messages in updateDisplay() function
     - Add more expression types by expanding the switch statement
     - Implement additional visual effects (blinking, scrolling, etc.)
*/
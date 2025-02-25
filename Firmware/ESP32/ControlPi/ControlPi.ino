
String receivedMessage = "";  // Variable to store the complete message
int piState = -1;
int desiredState = -1;
unsigned long startTime_TEST = 0;
unsigned long currentMs = 0;
unsigned long lastSendMs = 0;
bool FLAG_PI_STARTED = false;

#define LED_PIN 2

void setup() {
  // Start the Serial Monitor at a baud rate of 115200
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Check if data is available in the Serial buffer
  while (Serial.available()) {
    char incomingChar = Serial.read();  // Read each character from the buffer
    
    if (incomingChar == '\n') {  // Check if end of message

      if (receivedMessage.startsWith("State: ")) {        // If message is about pi state
          piState = receivedMessage.substring(7).toInt();   // Extract integer state

          if (FLAG_PI_STARTED == false && piState == 2) {       // If Pi just booted and ready in Idle allow other code to start
            FLAG_PI_STARTED = true;
            desiredState = piState;                       // Want to be in idle same time at start
            startTime_TEST = millis();           // (In test start timer until want to switch to running)
          }
      }

      receivedMessage = ""; // Clear the message buffer
    } else {
      // Append the character to the message string
      receivedMessage += incomingChar;
    }
  }

  currentMs = millis();

  // Testing stimulus based on time (will be replaced with RC)
  if (currentMs - startTime_TEST >= 3000 && desiredState == 2) {  // After three seconds in idle switch to run
    startTime_TEST = currentMs;
    desiredState = 0;     // Swap desired state to run
  }
  if (currentMs - startTime_TEST >= 15000 && desiredState == 0) {  // After 15 seconds in run switch to stop
    startTime_TEST = currentMs;
    desiredState = 1;     // Swap desired state to run
  }

  // When pi is not in desired state tell pi to change every 100 ms
  if (piState != desiredState && FLAG_PI_STARTED == true && currentMs - lastSendMs >= 100) {
    Serial.print("State: ");
    Serial.println(desiredState);
    lastSendMs = currentMs;

    //Test restart start time while not in correct state
    startTime_TEST = currentMs;
  }

  // Run ended so reset
  if ((piState == 1) && (desiredState == 1))
  {
    desiredState = -1;
    piState = -1;
    FLAG_PI_STARTED = false;
  }



}

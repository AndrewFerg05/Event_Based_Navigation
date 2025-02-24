int piState = 0, x, y, desiredState = 0;
String receivedMessage = "";

int RUN = 0, STOP = 1, IDLE = 2;
unsigned long lastStateChangeTime = 0;
bool timerIdle = 0;
bool timerRun = 0;
bool connectionConfirmed = 0;

void setup() {
    Serial.begin(115200);       //UART comms to Pi
}

void loop() {

    // Serial comms section

    // Read messages that have come in
    while (Serial.available()) {
    // Single message in format: "{state},{x},{y}\n"
    char incomingChar = Serial.read();  // Read next character in buffer
    
    if (incomingChar == '\n') {  // Check if end of message
      // Remove newline character if needed
      receivedMessage.trim(); 

      int firstComma = receivedMessage.indexOf(',');
      int secondComma = receivedMessage.indexOf(',', firstComma + 1);

      // Extract substrings
      piState = receivedMessage.substring(0, firstComma).toInt();
      x = receivedMessage.substring(firstComma + 1, secondComma).toInt();
      y = receivedMessage.substring(secondComma + 1).toInt();

      connectionConfirmed = 1;

      // Clear the message buffer for the next input
      receivedMessage = "";

    } else {
      receivedMessage += incomingChar; // Append char to message
    }
  }

  // Timer from first IDLE receive before saying to change state
  if (piState == IDLE && !timerIdle  && connectionConfirmed) {
        lastStateChangeTime = millis();  // Record the time when piState first becomes 0
        timerIdle = true;             // Mark that the timer has started
  }
  if (timerIdle && millis() - lastStateChangeTime >= 2000) {
        // 2 seconds have passed since piState was 0
        desiredState = RUN;
        timerIdle = 0;
  }

  // Timer from first RUN receive before saying to stop
  if (piState == RUN && !timerRun  && connectionConfirmed) {
        lastStateChangeTime = millis();  // Record the time when piState first becomes 0
        timerRun = true;             // Mark that the timer has started
  }
  if (timerRun && millis() - lastStateChangeTime >= 10000) {
        // 2 seconds have passed since piState was 0
        desiredState = STOP;
        timerRun = 0;
  }

  // Send message to tell what state pi should be in (for now based on time but in future use RC)
  if (piState != desiredState)
  {
    Serial.print("State: ");
    Serial.println(desiredState);
  }
}

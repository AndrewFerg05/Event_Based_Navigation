
String receivedMessage = "";  // Variable to store the complete message
String longmessage = "";
int count = 1;
#define LED_PIN 2

void setup() {
  // Start the Serial Monitor at a baud rate of 115200
  Serial.begin(115200);
  
  // Print an initial message to the Serial Monitor
  Serial.println("ESP32 is ready. Please enter a message:");
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  
  // Check if data is available in the Serial buffer
  while (Serial.available()) {
    char incomingChar = Serial.read();  // Read each character from the buffer
    
    if (incomingChar == '\n') {  // Check if the user pressed Enter (new line character)
      // Print the message
      digitalWrite(LED_PIN, HIGH);
      Serial.print("You sent: ");
      Serial.print(receivedMessage);
      Serial.print("\n");

      count++;

      Serial.print("Next iteration will be: ");
      Serial.print(String(count));
      Serial.print("\n");
      
      // Clear the message buffer for the next input
      longmessage+=receivedMessage;
      receivedMessage = "";

    } else {
      // Append the character to the message string
      receivedMessage += incomingChar;
    }
  }

}

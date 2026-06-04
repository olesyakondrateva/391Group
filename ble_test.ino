#include <ArduinoBLE.h>

#define BUFFER_SIZE 20
char currentCommand[BUFFER_SIZE + 1] = "BALANCE"; // tracks active state
// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("BLE-DEVICE-WHISKR");
  BLE.setDeviceName("BLE-DEVICE-WHISKR");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) { 
        int length = customCharacteristic.valueLength(); 
        const unsigned char* receivedData = customCharacteristic.value(); 
        char receivedString[length + 1]; 
        memcpy(receivedString, receivedData, length); 
        receivedString[length] = '\0'; 
        customCharacteristic.writeValue("Data received");
        strncpy(currentCommand, receivedString, BUFFER_SIZE); // ← save it
        Serial.println(currentCommand);
      }

      // Then below, always running every loop iteration:
      if (strcmp(currentCommand, "FORWARD") == 0) { 
        // keep fwd
      } 
      else if (strcmp(currentCommand, "REVERSE") == 0) {  
        //keep reversing 
      } 

      else if (strcmp(currentCommand, "LEFT") == 0) {  
        //keep turning left 
      } 

      else if (strcmp(currentCommand, "RIGHT") == 0) {  
        //keep turning right 
      } 

      else if (strcmp(currentCommand, "BALANCE") == 0) {  
        //keep balancing 
      } 
      
      else if (strcmp(currentCommand, "STOP") == 0) { 
        //implement stop }
      }
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }
}

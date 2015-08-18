// Art Car Drive-by-Wire
// Sender Module
// Input: Throttle lever (potentiometer), steering wheel (potentiometer)
// Garrett Mace

// Control ADC smoothing ratio
#define steerSmoothing 0.1
#define speedSmoothing 0.1
#define steerTrimSmoothing 0.05
#define brakeTrimSmoothing 0.05

// Steering input range
#define steerInMax 1023
#define steerInMin 0
#define steerInCenter 511
#define steerInDeadZone 5
#define steerWidth 500

// Braking input range
#define brakeInMax 546
#define brakeInMin 130
#define brakeInMaxDeadZone 20
#define brakeInMinDeadZone 10
#define brake25Point 250

// Throttle input range
#define throttleInMax 946
#define throttleInMin 630
#define throttleInMaxDeadZone 10
#define throttleInMinDeadZone 10

// Steering output range
#define steerMax 2000
#define steerMin 1000
#define steerDefault 1500

// Braking output range
#define brakeMax 2400
#define brakeMin 600
#define brakeDefault 0

// Throttle output range
#define throttleMax 255
#define throttleMin 0
#define throttleDefault 0

// I/O pin definitions
#define steerADC 3
#define speedADC 2
#define steerTrimADC 4
#define brakeTrimADC 5

#define enableRS485 2
#define statusLED 10

// Global variables
int steerValue = 1500;
int steerInCenterCal = 511;
int brakeValue = 180;
int throttleValue = throttleDefault;
float steerSmoothed = 0;
float speedSmoothed = 0;
float steerTrimSmoothed = 0;
float brakeTrimSmoothed = 0;

// Receive buffer size
#define recvMax 13
int recvIndex = 0;
char recvBuffer[recvMax] = {0};
byte sendBuffer[13] = {0};

// Communcation state definitions
  #define IDLE 0
  #define RECEIVE 1
  #define TRANSMIT 2
  #define RXDONE 3
  #define TXDONE 4
  #define ERROR 5

int packetState = IDLE;


// Initialize sequence
void setup()
{

  // Activate serial port
  Serial.begin(57600);

  // Communication status LED
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED,HIGH);

  // RS485 flow control pin
  pinMode(enableRS485, OUTPUT);
  digitalWrite(enableRS485, LOW);
  
  
  // Calibrate steering on startup
  int steerCalAvg;
  for (int i = 0; i < 15; i++) {
    steerCalAvg += analogRead(steerADC);
  }
  steerInCenterCal = steerCalAvg / 15;
 
}





// Run CRC on data to be sent
byte doChecksum(void) {
 byte crc = 0, i;
 for (i = 0; i < 12; i++) {
   crc = crc + sendBuffer[i];
 } 
 return crc;
}


// Check receive buffer for valid packet and store the data
void checkRS485()
{
  byte tempByte;
  byte serialFloodDetect = 0;
  
  while (Serial.available() > 0 && serialFloodDetect < 50)
  {
    
    tempByte = Serial.read();
    serialFloodDetect++;
   
    if (packetState == RECEIVE) { 
      // end of packet
      if (tempByte == 0x03) {
        // Quick and dirty check for ACK packet from receiver   
        if (recvBuffer[recvIndex-2] == char('O') && recvBuffer[recvIndex-1] == char('K')) {
          packetState = RXDONE;
        } else {
          packetState = ERROR;
        }
      // receive data
      } else if (recvIndex < recvMax) {
        recvBuffer[recvIndex] = tempByte;
        recvIndex++;
      } else {
        packetState = ERROR;
      }
     
    }
    
    // start of new packet
    if (tempByte == 0x02) {
      recvIndex = 0;
      packetState = RECEIVE;
    }
    
  }
  
  serialFloodDetect = 0;
  
}

// Pad number with leading zeros if needed
String leadZeroes(int value, int minlength) {
  String tempString = String(value, DEC);
  int tempLength = tempString.length();
  if (tempLength < minlength) {
    String zeroFill = String("");
    for (int i = 0; i < (minlength - tempLength); i++) zeroFill += "0";
    tempString = zeroFill + tempString;
  }
  return tempString;
}

// Generate the control packet
void buildPacket() {

  String tempString = leadZeroes(steerValue,4) + leadZeroes(brakeValue,4) + leadZeroes(throttleValue,4);
 
  tempString.getBytes(sendBuffer, 13);

  byte checksum = doChecksum();
  if (checksum < 16) {
    tempString = tempString + char('0') + String(doChecksum(),HEX);
  } else {
    tempString = tempString + String(doChecksum(),HEX);
  }    
 
  String packetString = char(0x02) + tempString + char(0x03);

  digitalWrite(enableRS485,HIGH);   // enable RS485 transmit
  Serial.print(packetString);       // send serial packet
  Serial.flush();                   // wait for transmission to complete
  digitalWrite(enableRS485,LOW);    // disable RS485 transmit
  
}

// Sample and smooth analog input values
void readAnalogs() {
 
  steerSmoothed = steerSmoothed * (1.0 - steerSmoothing) + analogRead(steerADC) * steerSmoothing;
  speedSmoothed = speedSmoothed * (1.0 - speedSmoothing) + analogRead(speedADC) * speedSmoothing;

  steerTrimSmoothed = steerTrimSmoothed * (1.0 - steerTrimSmoothing) + analogRead(steerTrimADC) * steerTrimSmoothing;
  brakeTrimSmoothed = brakeTrimSmoothed * (1.0 - brakeTrimSmoothing) + analogRead(brakeTrimADC) * brakeTrimSmoothing;
  
}

// Process analog values and convert to command values
void calcOutputs() {

  // Apply adjustable brake offset from trimpot
  int brakeOffset =  (brakeTrimSmoothed - 511) / 8;

  // When throttle is below a certain point, apply brakes
  if (speedSmoothed > brakeInMax-brakeInMaxDeadZone) {
    brakeValue = brakeMax;
  } else if (speedSmoothed < brakeInMin+brakeInMinDeadZone) {
    brakeValue = brakeMin;
  } else if (speedSmoothed <= brakeInMax && speedSmoothed >= brakeInMax-brake25Point) {
    brakeValue = map(speedSmoothed, brakeInMax-brake25Point, brakeInMax, brakeMax*0.80, brakeMax) + brakeOffset;
  } else if (speedSmoothed < brakeInMax-brake25Point && speedSmoothed >= brakeInMin) {
    brakeValue = map(speedSmoothed, brakeInMin, brakeInMax-brake25Point, brakeMin, brakeMax*0.80) + brakeOffset;
  } 

  // When throttle is above a certain point, apply throttle
  if (speedSmoothed > throttleInMax-throttleInMaxDeadZone) {
    throttleValue = throttleMax;
  } else if (speedSmoothed < throttleInMin+throttleInMinDeadZone) {
    throttleValue = throttleMin;
  } else if (speedSmoothed <= throttleInMax && speedSmoothed >= throttleInMin) {
    throttleValue = map(speedSmoothed, throttleInMin+throttleInMinDeadZone, throttleInMax-throttleInMaxDeadZone, throttleMin, throttleMax);
  }
  
  if (throttleValue >= 255) throttleValue = 255;
  if (throttleValue <= 0) throttleValue = 0;
  
  // Apply adjustable steering offset from trimpot
  int steerOffset =  (steerTrimSmoothed - 511) / 2;
  
  // Generate steering values  
  if (steerSmoothed >= steerInMax) {
    steerValue = steerMax + steerOffset;
  } else if (steerSmoothed <= steerInMin) {
    steerValue = steerMin + steerOffset;
  } else if (steerSmoothed <= steerInCenterCal+steerInDeadZone && steerSmoothed >= steerInCenterCal-steerInDeadZone) {
    steerValue = steerDefault + steerOffset;
  } else if (steerSmoothed < steerInCenterCal+steerWidth && steerSmoothed > steerInCenterCal-steerWidth) {
    steerValue = map(steerSmoothed, steerInCenterCal-steerWidth, steerInCenterCal+steerWidth, steerMin, steerMax) + steerOffset;
  }
  
}




long activityLEDTimer = 0;
long activityPacketTimer = 0;
long activityAnalogTimer = 0;
int activityFlag = 0;

// Main control loop
void loop()
{

  checkRS485();

  long currentMillis;

  // Read analog values every 10 milliseconds
  currentMillis = millis();
  if (currentMillis - activityAnalogTimer > 10) {
    readAnalogs();
    activityAnalogTimer = currentMillis;
  }

  // Send data packet every 50 milliseconds
  currentMillis = millis();
  if (currentMillis - activityPacketTimer > 50) {
    calcOutputs();
    buildPacket();
    activityPacketTimer = currentMillis;
  }

  // Handle packet state and status LED
  if (packetState == RXDONE) {
    packetState = IDLE;
    digitalWrite(statusLED, LOW);
    activityFlag = 1;
    activityLEDTimer = millis();
 }

  // Turn status LED back on after 10 milliseconds
  if (activityFlag == 1 && (millis() - activityLEDTimer) > 10) {
    activityFlag = 0;
    digitalWrite(statusLED, HIGH);
  }
  
  
}

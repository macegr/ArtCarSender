// Art Car 2011
// Sender Module
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

// Initialize sequence
void setup()
{
  
  Serial.begin(57600);
  
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED,HIGH);
  
  pinMode(enableRS485, OUTPUT);
  
  
  int steerCalAvg;
  
  for (int i = 0; i < 15; i++) {
    steerCalAvg += analogRead(steerADC);
  }
  
  steerInCenterCal = steerCalAvg / 15;
  
  
}


#define recvMax 13
int recvIndex = 0;
char recvBuffer[recvMax] = {0};

  #define IDLE 0
  #define RECEIVE 1
  #define TRANSMIT 2
  #define RXDONE 3
  #define TXDONE 4
  #define ERROR 5

int packetState = IDLE;


byte sendBuffer[13] = {0};

byte doChecksum(void) {
 byte crc = 0, i;
 for (i = 0; i < 12; i++) {
   crc = crc + sendBuffer[i];
 } 
 return crc;
}


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


void wait_for_done()
{
  char timeout = 15;  // delay in 100 us units
  
  UCSR0A |= 1<<TXC0;  // clear flag!
  while((UCSR0A&(1<<TXC0)) == 0) {
     delayMicroseconds(100);
     if (--timeout <= 0)
        break;
  }
}

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


void readAnalogs() {
 
  steerSmoothed = steerSmoothed * (1.0 - steerSmoothing) + analogRead(steerADC) * steerSmoothing;
  speedSmoothed = speedSmoothed * (1.0 - speedSmoothing) + analogRead(speedADC) * speedSmoothing;

  steerTrimSmoothed = steerTrimSmoothed * (1.0 - steerTrimSmoothing) + analogRead(steerTrimADC) * steerTrimSmoothing;
  brakeTrimSmoothed = brakeTrimSmoothed * (1.0 - brakeTrimSmoothing) + analogRead(brakeTrimADC) * brakeTrimSmoothing;
  
}

void calcOutputs() {

  int brakeOffset =  (brakeTrimSmoothed - 511) / 8;
  
  if (speedSmoothed > brakeInMax-brakeInMaxDeadZone) {
    brakeValue = brakeMax;
  } else if (speedSmoothed < brakeInMin+brakeInMinDeadZone) {
    brakeValue = brakeMin;
  } else if (speedSmoothed <= brakeInMax && speedSmoothed >= brakeInMax-brake25Point) {
    brakeValue = map(speedSmoothed, brakeInMax-brake25Point, brakeInMax, brakeMax*0.80, brakeMax) + brakeOffset;
  } else if (speedSmoothed < brakeInMax-brake25Point && speedSmoothed >= brakeInMin) {
    brakeValue = map(speedSmoothed, brakeInMin, brakeInMax-brake25Point, brakeMin, brakeMax*0.80) + brakeOffset;
  } 
  
  if (speedSmoothed > throttleInMax-throttleInMaxDeadZone) {
    throttleValue = throttleMax;
  } else if (speedSmoothed < throttleInMin+throttleInMinDeadZone) {
    throttleValue = throttleMin;
  } else if (speedSmoothed <= throttleInMax && speedSmoothed >= throttleInMin) {
    throttleValue = map(speedSmoothed, throttleInMin+throttleInMinDeadZone, throttleInMax-throttleInMaxDeadZone, throttleMin, throttleMax);
  }
  
  if (throttleValue >= 255) throttleValue = 255;
  if (throttleValue <= 0) throttleValue = 0;
  
  
  int steerOffset =  (steerTrimSmoothed - 511) / 2;
  
  //Serial.println(throttleValue);
  
    
  if (steerSmoothed >= steerInMax) {
    steerValue = steerMax + steerOffset;
  } else if (steerSmoothed <= steerInMin) {
    steerValue = steerMin + steerOffset;
  } else if (steerSmoothed <= steerInCenterCal+steerInDeadZone && steerSmoothed >= steerInCenterCal-steerInDeadZone) {
    steerValue = steerDefault + steerOffset;
  } else if (steerSmoothed < steerInCenterCal+steerWidth && steerSmoothed > steerInCenterCal-steerWidth) {
    steerValue = map(steerSmoothed, steerInCenterCal-steerWidth, steerInCenterCal+steerWidth, steerMin, steerMax) + steerOffset;
  }
  
  
  //Serial.print(steerSmoothed); Serial.print(" ");
  //Serial.println(steerValue);
  
}




long activityLEDTimer = 0;
long activityPacketTimer = 0;
long activityAnalogTimer = 0;
int activityFlag = 0;

// Main control loop
void loop()
{

  checkRS485();

  if (millis() > activityAnalogTimer) {
    readAnalogs();
    activityAnalogTimer = millis() + 5;
  }

  if (millis() > activityPacketTimer) {
    calcOutputs();
    buildPacket();
    activityPacketTimer = millis() + 50;
  }

  if (packetState == RXDONE) {
    packetState = IDLE;
    digitalWrite(statusLED, LOW);
    activityFlag = 1;
    activityLEDTimer = millis()+10;
 }

  
  if (activityFlag == 1 && millis() > activityLEDTimer) {
    activityFlag = 0;
    digitalWrite(statusLED, HIGH);
  }
  
  
}

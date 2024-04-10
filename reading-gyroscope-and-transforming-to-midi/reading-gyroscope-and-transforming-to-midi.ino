
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


#include "MIDIUSB.h"    // include the MIDIUSB library
// https://www.arduino.cc/reference/en/libraries/midiusb/


/*
 * median library is used to smooth flickering values
 */
#include "RunningMedian.h"
const int medianSize = 33;
RunningMedian MIDIVal1Array = RunningMedian(medianSize);
int MIDIVal = 0;       
int prevMIDIVal = 0; 
int MIDIChan = 0; 

/*
#include "MIDI.h"  // this is to send midi notes but have nothing to do with midi usb so dont use
MIDI_CREATE_DEFAULT_INSTANCE();*/
int NOTE_NUMBER = 60;
int prevMIDINote = 60; 


MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  //MIDI.begin(); // this is to send midi notes but not through usb 
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);     // 31250 for MIDI   and 115200 for midiUSB ?? 
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
 /* while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again  */ 
 //  load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


// a function that creates a controller change 
void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}
/*
void playNote ( byte channel, byte note) {
    midiEventPacket_t noteOn = {0x09, 0x90 | channel , note, 40};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
  delay(200);
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, note, 40};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}*/


void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //X-axis or roll
   // Serial.print("accx:");
   // Serial.println(ypr[2] * 180 / M_PI);
    //y-axis or pitch
    //Serial.print("accy:");
   // Serial.println (ypr[1] * 180 / M_PI);
    //z-axis or yaw
   // Serial.print("accz:");
  // int myInt = (ypr[0] * 180 / M_PI)+180;
  //  Serial.println(myInt);
  //  Serial.print("aababa");
  }
 

int MIDIVal = map((ypr[2] * 180 / M_PI),-90,90, 0, 127);;     
 MIDIVal1Array.add(MIDIVal);            // write to median array
 MIDIVal = MIDIVal1Array.getMedian();   // read from median array

// Serial.println(MIDIVal);
// Serial.print("midi value::");
//delay(5);

 if (MIDIVal != prevMIDIVal) {          // if controller has changed from previous pass...
  controlChange(MIDIChan, 1, MIDIVal);  // send to midi chanel 1, to the CC 1, the MIDIVal
  MidiUSB.flush();
  delay(1);                             // a delay is needed between MIDI flush events
 }
 prevMIDIVal = MIDIVal;  // 

/*
int NOTE_NUMBER = map((ypr[2] * 180 / M_PI),-90,90, 0, 127);;  
 MIDIVal1Array.add(NOTE_NUMBER);            // write to median array
 NOTE_NUMBER = MIDIVal1Array.getMedian(); 

 if (NOTE_NUMBER != prevMIDINote) {
   playNote(MIDIChan, NOTE_NUMBER);
   delay(50);  
 }
 prevMIDINote = NOTE_NUMBER;*/


}

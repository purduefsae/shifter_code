// libraries
#include <FlexCAN_T4.h>

// define CAN receiver
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_0;

// pin defs
const int motcon_pwm = 2;
const int enc_abs = 3;
const int enc_a = 4;
const int enc_b = 5;

// constants
// shift dist calculated from 4096 events per revolution of the encoder, a 4:1 reduction stage, and an assumed 10 degs for a shift
const int return_length = 1;
unsigned long shift_limit = 500;
volatile unsigned long shift_time = 0;
unsigned long shift_start = 0;

// sketch vars
volatile int enc_position = 0;
int enc_home = 0;
int enc_state = 0;
//int downshift_position = 2900;
int downshift_position = 750;
//int upshift_position = 600;
int upshift_position = -1050;
int shift_state = 0; // -1 downshift, 0 no shift, 1 upshift

// interrupt handlers
void interrupt_a() {
  switch(enc_state) {
    case 0:
      enc_state = 1;
      enc_position--;
      break;
    case 1:
      enc_state = 0;
      enc_position++;
      break;
    case 2:
      enc_state = 3;
      enc_position++;
      break;
    case 3:
      enc_state = 2;
      enc_position--;
      break;
  }
}

void interrupt_b() {
  switch(enc_state) {
    case 0:
      enc_state = 2;
      enc_position++;
      break;
    case 1:
      enc_state = 3;
      enc_position--;
      break;
    case 2:
      enc_state = 0;
      enc_position--;
      break;
    case 3:
      enc_state = 1;
      enc_position++;
      break;
  }
}

void receive_shift_mess(const CAN_message_t &msg) {
  if (msg.id == 160) {
    //Serial.println(msg.buf[0] != 0);
    if (msg.buf[0] != 0 and shift_state == 0) {
      shift_state = 1;
      Serial.println("upshift");
    } else if (msg.buf[1] != 0 and shift_state == 0) {
      shift_state = -1;
      Serial.println("downshift");
    }
  }
}

// general functions
void init_position() {
  // set state
  if (digitalRead(enc_a)) {
    enc_state = enc_state + 1;
  }

  if (digitalRead(enc_b)) {
    enc_state = enc_state + 2;
  }
  
  // set abs position
  //enc_position = pulseIn(enc_abs, LOW);
  enc_position = 0;
  enc_home = enc_position;
}

void setup() {
  // put your setup code here, to run once:
  // set pin definitions
  delay(1000);
  pinMode(motcon_pwm, OUTPUT);
  pinMode(enc_abs, INPUT);
  pinMode(enc_a, INPUT);
  pinMode(enc_b, INPUT);
  pinMode(13, OUTPUT); // board on pin. just don't use 13 unless you have to
  digitalWrite(13, HIGH);
  
  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(enc_a), interrupt_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_b), interrupt_b, CHANGE);

  // setup vars
  init_position();

  // set pulse characteristics
  analogWriteFrequency(motcon_pwm, 200);

  Serial.begin(9600);
  
  // setup CAN receive stuff
  can_0.begin();
  can_0.setBaudRate(1000000);
  can_0.setMaxMB(8);
  can_0.enableFIFO();
  can_0.enableFIFOInterrupt();
  can_0.enableMBInterrupts();
  can_0.onReceive(FIFO, receive_shift_mess);
  can_0.mailboxStatus();
  can_0.setFIFOFilter(0, 0x0a0, STD);
}

void loop() {
  // can logic
  can_0.events();
  
  // motor logic
  switch(shift_state) {
    case 2:
      delay(500);
      shift_state = 0;
      break;
    case -1:
      analogWrite(motcon_pwm, 103);
      shift_start = millis();
      shift_time = 0;
      while (enc_position < downshift_position && shift_time < shift_limit) {
        shift_time = millis() - shift_start; 
        //Serial.print(shift_time);
      }
      analogWrite(motcon_pwm, 77);
      delay(100);
      shift_state = 2;
      break;
    case 1:
      analogWrite(motcon_pwm, 51);
      shift_start = millis();
      shift_time = 0;
      while (enc_position > upshift_position && shift_time < shift_limit) {
        shift_time = millis() - shift_start;
      }
      analogWrite(motcon_pwm, 77);
      delay(100);
      shift_state = 2;
      break;
    case 0:
      analogWrite(motcon_pwm, 77);
      //Serial.println(String(enc_position));
      //delay(100);
  }
}

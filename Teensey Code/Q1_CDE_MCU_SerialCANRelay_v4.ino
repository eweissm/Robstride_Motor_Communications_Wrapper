#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// Serial Communication - Match Python SerialCommunicator settings
const uint8_t syncByte = 0xAA;
const uint8_t MotorResponseID = 0x2;
const uint32_t host_id = 0xFD;  // Host's CAN ID

const uint16_t numMotors = 1;
const byte motorIds[numMotors] = { 0x0E};



//assign motors to specific can buses based on wiring
const byte Can1_Ids[1] = { 0x0E};
const byte Can2_Ids[0] = {};
const byte Can3_Ids[0] = {  };

const uint16_t python_FrameSize = 12; 
const uint16_t CAN_FrameSize = 12; 
const uint16_t output_FrameSize = 13; 

const uint16_t expectedBytes_from_python = python_FrameSize * numMotors;  //4 for ext ID plus 8 data
const uint16_t expectedBytes_to_python = output_FrameSize * numMotors;  //4 for ext ID plus 8 data

byte DataBytes[expectedBytes_from_python];
byte ReplyBytes[expectedBytes_to_python];

// CAN setup
CAN_message_t rxmsg;
bool MotorRepliesRecieved[numMotors];  // array which stores if all the motors have replied after can command was sent. When all bools are true, we will write over serial the responses.
bool msgSentFlag = false;
uint8_t noResponseCounter = 0;
const uint8_t noResponsethreshold = 100;  //number of noResponseCounts before motors are falgged as disconnected


void sendCANMessage(uint32_t id, byte *data);
void canSniff(const CAN_message_t &msg);

void setup() {
  //start serial coms
  Serial.begin(2000000);
  while (!Serial)
    ;  // Wait for Serial to initialize
  Serial.setTimeout(5);
  //initialize recieved data array
  for (size_t i = 0; i < expectedBytes_from_python; ++i) {
    DataBytes[i] = 0x00;
  }

  //initialize arrays
  for (size_t i = 0; i < numMotors; ++i) {
    MotorRepliesRecieved[i] = 0;
  }

  //setup CAN
  Can1.begin();
  Can1.setBaudRate(1000000);  // 1Mbps as per protocol
  Can1.setMaxMB(32);
  Can1.enableFIFO();
  Can1.mailboxStatus();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(canSniff);
  delay(100);

  // Clear any pending messages
  while (Can1.read(rxmsg)) {
    delay(1);
  }


  //setup CAN
  Can2.begin();
  Can2.setBaudRate(1000000);  // 1Mbps as per protocol
  Can2.setMaxMB(32);
  Can2.enableFIFO();
  Can2.mailboxStatus();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(canSniff);
  delay(100);

  // Clear any pending messages
  while (Can2.read(rxmsg)) {
    delay(1);
  }


  //setup CAN
  Can3.begin();
  Can3.setBaudRate(1000000);  // 1Mbps as per protocol
  Can3.setMaxMB(32);
  Can3.enableFIFO();
  Can3.mailboxStatus();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);
  delay(100);

  // Clear any pending messages
  while (Can3.read(rxmsg)) {
    delay(1);
  }
}

void loop() {
  Can1.events();
  Can2.events();
  Can3.events();

  // Look for sync byte, then read exactly expectedBytes with timeout
  while (Serial.available()) {
    int b = Serial.read();
    if (b < 0) break;

    if ((uint8_t)b == syncByte) {
      size_t n = Serial.readBytes(DataBytes, expectedBytes_from_python);  // blocks up to timeout
      if (n != expectedBytes_from_python) {
        // timed out / partiexpectedBytes_from_pythonal frame -> resync by scanning to next sync byte
        continue;
      }

      // Dispatch CAN frames immediately (skip all-zero init)
      if (DataBytes[0] != 0x00) {
        for (uint16_t i = 0; i < numMotors; ++i) {
          const uint8_t *parsed = &DataBytes[i * CAN_FrameSize];
          uint32_t extId = extractExtId(parsed);  // bytes 0..3
          const uint8_t *payload = parsed + 4;    // bytes 4..11

          sendCANMessage(extId, payload);
          msgSentFlag = true;

          // delayMicroseconds(200);
          delayMicroseconds(50);
          Can1.events();
          Can2.events();
          Can3.events();
        }
      }


      // After TX, we’ll fall through to the reply check below
      break;
    }
    // byte wasn’t sync; keep scanning
  }

  //lets check if all the motors have replied.
  noInterrupts();
  bool all_Motors_replied = 1;
  for (size_t i = 0; i < numMotors; i++)
    if (!MotorRepliesRecieved[i]) {
      all_Motors_replied = 0;
      break;
    }
  interrupts();

  if (all_Motors_replied) {
    // if all the motors have replied, then send out msg like normal. Reset noResponseCounter and msgSentFlag
    noInterrupts();
    Serial.write(syncByte);
    Serial.write(ReplyBytes, expectedBytes_to_python);

    for (size_t i = 0; i < numMotors; ++i) {
      MotorRepliesRecieved[i] = 0;  //reset MotorRepliesRecieved
    }

    interrupts();

    msgSentFlag = false;
    noResponseCounter = 0;
  } else if (msgSentFlag) {
    // if all motors have not replied AND we are expecting a response

    noResponseCounter++;  //increment counter
    delayMicroseconds(10);

    if (noResponseCounter > noResponsethreshold) {
      noResponseCounter = 0;
      //if we have not heard back from all the motors in noResponsethreshold cycles, we assume any motors in all_Motors_replied marked false must have an error
      for (size_t i = 0; i < numMotors; ++i) {
        if (MotorRepliesRecieved[i] == 0) {
          ReplyBytes[i * output_FrameSize] = 0xFF;             //set msg cmd to 0xFF
          ReplyBytes[i * output_FrameSize + 1] = motorIds[i];  //set msg cmd to 0xFF
          MotorRepliesRecieved[i] = 1;
        }
      }

      // //send out the edited msg
      // Serial.write(syncByte);
      // noInterrupts();
      // Serial.write(ReplyBytes, expectedBytes);
      // interrupts();
    }
  }
}


static inline bool sendCANBlocking(const CAN_message_t &msg, uint32_t timeout_us = 2000) {
  uint32_t start = micros();

  uint32_t id = msg.id;
  uint8_t motorID = (uint8_t)((id >> 0) & 0xFF);

  if (elementOf(Can1_Ids, motorID, sizeof(Can1_Ids))) {

    while (!Can1.write(msg)) {
      Can1.events();
      if ((micros() - start) > timeout_us) return false;  // give up cleanly

      // no delay needed; the bus/ISR frees space within a few hundred µs
    }
  } else if (elementOf(Can2_Ids, motorID, sizeof(Can2_Ids))) {

    while (!Can2.write(msg)) {
      Can2.events();
      if ((micros() - start) > timeout_us) return false;  // give up cleanly

      // no delay needed; the bus/ISR frees space within a few hundred µs
    }
  } else if (elementOf(Can3_Ids, motorID, sizeof(Can3_Ids))) {

    while (!Can3.write(msg)) {
      Can3.events();
      if ((micros() - start) > timeout_us) return false;  // give up cleanly

      // no delay needed; the bus/ISR frees space within a few hundred µs
    }
  } else {
  }

  return true;
}

bool elementOf(byte *list, byte element, uint16_t listLength) {
  bool flag = 0;
  for (uint16_t i = 0; i < listLength; i++) {
    if (list[i] == element) {
      flag = 1;
      break;
    }
  }
  return flag;
}

void sendCANMessage(uint32_t id, byte *data) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = id & 0x1FFFFFFF;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = data[i];
  (void)sendCANBlocking(msg);  // handle failure if you want
}

//pulls first 4 bytes of array to build extended ID
static inline uint32_t extractExtId(const uint8_t *b) {
  uint32_t id = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | ((uint32_t)b[3] << 0);
  // Mask to 29 bits just in case
  return id;
}

void canSniff(const CAN_message_t &msg) {
  // ID -> first 4 bytes (big-endian). Mask to 29 bits for safety.
  uint32_t id = (msg.id);

  if ((uint8_t)((id >> 24) & 0xFF) == (uint8_t)MotorResponseID) {

    byte motorID = (uint8_t)((id >> 8) & 0xFF);
    
    //find motor id index
    int index = -1;
    for (uint16_t i = 0; i < numMotors; i++) {
      if (motorID == motorIds[i]) {
        index = i;
        break;
      }
    }
    //if we dont find any motor with the same ID, dont continue
    if (index != -1) {
      const int OFFSET_CMD = 0;     // ReplyBytes[index*FrameSize + 0]
      const int OFFSET_ID = 1;      // ReplyBytes[index*FrameSize + 1]
      const int OFFSET_STATE = 2;      // ReplyBytes[index*FrameSize + 1]
      const int OFFSET_FLOAT = 3;   // 4 bytes: 2,3,4,5
      const int OFFSET_REMAIN = 7;  // remaining bytes: 6..11 (6 bytes)

      // fill in ReplyBytes for the specific motor
      ReplyBytes[index * output_FrameSize + OFFSET_CMD] = (uint8_t)((id >> 24) & 0xFF);  //cmd ID (0x02)
      ReplyBytes[index * output_FrameSize + OFFSET_ID] = (uint8_t)((id >> 8) & 0xFF);    //motor id
      ReplyBytes[index * output_FrameSize + OFFSET_STATE] = (uint8_t)((id >> 16) & 0xFF);    //motor state
      // Data -> next 8 bytes (pad if needed)
      uint8_t tempResponseData[8] = { 0 };
      for (uint8_t i = 0; i < 8; i++) {
        tempResponseData[i] = (i < msg.len) ? msg.buf[i] : 0x00;
      }
      //we want to take the first two bytes of data, convert it from coil angle to output angle and then pass along the rest of the message

      // convert first two bytes to int and then to float from -4pi to pi
      uint16_t raw = ((uint16_t)tempResponseData[0] << 8) | tempResponseData[1];
      float angle = ((float)raw / 65535.0f) * (8.0f * PI) - 4.0f * PI;


      //convert float back to byte
      uint8_t AngleBytes[4];
      memcpy(AngleBytes, &angle, sizeof(float));

      for (uint8_t b = 0; b < 4; b++) {
        ReplyBytes[index * output_FrameSize + OFFSET_FLOAT + b] = AngleBytes[b];
      }

      // append the remaining 6 bytes (tempResponseData[2..7]) at OFFSET_REMAIN
      for (uint8_t b = 0; b < 6; b++) {
        ReplyBytes[index * output_FrameSize + OFFSET_REMAIN + b] = tempResponseData[2 + b];
      }

      MotorRepliesRecieved[index] = 1;  // mark that this reply is correct
    }
  } else if ((uint8_t)((id >> 24) & 0xFF) == (uint8_t)0x11) {
    byte motorID = (uint8_t)((id >> 8) & 0xFF);
  
    //find motor id index
    int index = -1;
    for (uint16_t i = 0; i < numMotors; i++) {
      if (motorID == motorIds[i]) {
        index = i;
        break;
      }
    }
    //if we dont find any motor with the same ID, dont continue
    if (index != -1) {
      const int OFFSET_CMD = 0;     // ReplyBytes[index*FrameSize + 0]
      const int OFFSET_ID = 1;      // ReplyBytes[index*FrameSize + 1]
      const int OFFSET_STATE = 2;
      const int OFFSET_FLOAT = 3;   // 4 bytes: 2,3,4,5
      const int OFFSET_REMAIN = 7;  // remaining bytes: 6..11 (6 bytes)

      // fill in ReplyBytes for the specific motor
      ReplyBytes[index * output_FrameSize + OFFSET_CMD] = (uint8_t)((id >> 24) & 0xFF);  //cmd ID (0x02)
      ReplyBytes[index * output_FrameSize + OFFSET_ID] = (uint8_t)((id >> 8) & 0xFF);    //motor id
      ReplyBytes[index * output_FrameSize + OFFSET_STATE] = (uint8_t)((id >> 16) & 0xFF);    //motor state


      for (uint8_t b = 0; b < 10; b++) {
        ReplyBytes[index * output_FrameSize + OFFSET_FLOAT + b] = 0x00;
      }

      MotorRepliesRecieved[index] = 1;  // mark that this reply is correct
    }
  }
}

uint32_t buildEID(uint8_t cmd, uint8_t host, uint8_t motor) {
  return (uint32_t(cmd) << 24) | (0 << 16) | (uint32_t(host) << 8) | (uint32_t(motor));
}

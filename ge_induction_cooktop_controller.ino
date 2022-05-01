/*
 * Arduino-based Controller Firmware for GE Induction Cooktops
 * 
 * Design tips:
 * - Use pull-down resistors and decoupling capacitors on the analog pot inputs to prevent erratic behavior.
 * - Tie the personality pin high if you are controlling a 36 inch cooktop with 5 coils and 3 generator boards.
 * - To connect to the single-wire half-duplex serial bus the generator boards use, connect a bus transceiver. Refer to the following for design tips: https://github.com/wfang2002/Full-Half-Duplex-Adapter
 * 
 * Note: you MUST use an Arduino-compatible board that supports at least two hardware serial busses, or change Serial or Serial2 to use SoftwareSerial.
 * 
 * Written by Joshua Currier (doitaljosh)
 * 05/01/2022
 */

/*
 * GEA addresses for the Arduino and the power boards
 */
#define LOCAL_ADDR 0x87
#define GEN1_ADDR 0x88
#define GEN2_ADDR 0x89
#define GEN3_ADDR 0x8a

/*
 * GPIO pin configuration. Change these to match your board!
 */

int heartbeatLed = 2; // Toggles every time the heartbeat increments

int pot1Pin = 25; // Rear left burner control knob
int pot2Pin = 26; // Front left burner control knob
int pot3Pin = 34; // Center burner control knob
int pot4Pin = 35; // Front right burner control knob
int pot5Pin = 32; // Rear right burner control knob
int personalitySelPin = 33; // 0: 4 burners, 1: 5 burners

int potPins[5] = {pot1Pin, pot2Pin, pot3Pin, pot4Pin, pot5Pin};

/* 
 * The GE induction generator boards support 20 power levels. 
 * These must be mapped to ADC values from the pots. 
 */
int minPowerSteps = 0;
int maxPowerSteps = 19;

int numberOfCoils;

typedef enum {
  GEA_ESC = 0xe0, // Escape
  GEA_ACK = 0xe1, // Acknowledge
  GEA_SOF = 0xe2, // Start of frame
  GEA_EOF = 0xe3 // End of frame
} GeaHeaderBytes;

/* 
 * Data structure of a basic GEA packet header
 */
typedef struct {
  uint8_t sof;
  uint8_t destination;
  uint8_t length;
  uint8_t source;
} __attribute__((__packed__))GeaMessageHeader_t;

/*
 * Command codes for the generator boards
 */
typedef enum {
  CMD_GET_SW_VERSION=0x01,
  CMD_SET_BOARD_CONFIG=0x26,
  CMD_SET_PWR_LEVELS=0x28,
  CMD_GET_CURRENTS=0x29,
  CMD_GET_STATUS=0x9e
} GeaCommandList;

/*
 * Coil power profiles
 */
typedef enum {
  COIL_TYPE_NONE=0x00,
  COIL_TYPE_1800_WATT=0x01,
  COIL_TYPE_2500_WATT=0x02,
  COIL_TYPE_3200_WATT=0x03,
  COIL_TYPE_3700_WATT=0x04
} CoilProfileId;

/*
 * Personality IDs
 */
typedef enum {
  PERSONALITY_FOUR_COILS=0,
  PERSONALITY_FIVE_COILS=1
} PersonalityId;

/*
 * 16-bit CRC polynomial table conforming to the GEA spec
 */
static const uint16_t crc16Table[] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/*
 * @brief Calculate a CRC16 given a data buffer and size
 */
uint16_t CalculateCrc16(char *dataToCheck, int sizeOfData) {
  uint16_t crc16;
  char *ptr;
  size_t a;

  crc16 = 0xe300;
  ptr = dataToCheck;

  if (ptr != NULL) {
    for(a=0; a<sizeOfData; a++) {
      crc16 = (crc16 >> 8) ^ crc16Table[(crc16 ^ (uint16_t)*ptr++) & 0x00ff];
    }
  }
  return crc16; 
}

/*
 * @brief Check if a given value is between a defined minimum and maximum 
 */
bool withinRange(int value, int minimum, int maximum) {
  return (minimum <= value && value <= maximum);
}

/*
 * @brief Check if a byte needs to be escaped.
 */
bool isEscaped(uint8_t value) {
  switch(value) {
    case GEA_ESC:
    case GEA_ACK:
    case GEA_SOF:
    case GEA_EOF:
      return true;
    default:
      return false;
  }
}

/*
 * @brief Initialize a generator board at an address and tell it what type of coils are connected
 */
int initSingleGenerator(uint8_t address, uint8_t profile1, uint8_t profile2) {
  GeaCommandList cmd;
  
  typedef struct {
    GeaMessageHeader_t header;
    uint8_t command;
    uint8_t coil1_profile;
    uint8_t coil2_profile;
  } __attribute__((__packed__))BoardConfigMsg_t;

  BoardConfigMsg_t msg;
  cmd = CMD_SET_BOARD_CONFIG;

  msg.header.sof = GEA_SOF;
  msg.header.destination = address;
  msg.header.length = 0x0a;
  msg.header.source = LOCAL_ADDR;
  msg.command = cmd;
  msg.coil1_profile = profile1;
  msg.coil2_profile = profile2;

  int txBufferSize = (sizeof(BoardConfigMsg_t) + 4);
  char txBuffer[txBufferSize];

  memcpy(txBuffer, &msg, txBufferSize);

  uint16_t crc16 = CalculateCrc16(txBuffer, (txBufferSize - 4));

  txBuffer[txBufferSize - 4] = crc16 & 0xff;
  txBuffer[txBufferSize - 3] = crc16 >> 8;
  txBuffer[txBufferSize - 2] = GEA_EOF;
  txBuffer[txBufferSize - 1] = GEA_ACK;

  for(int i=0; i<txBufferSize; i++) {
    Serial2.print(txBuffer[i]);
  }

  return 0;
}

/*
 * @brief Update the generator boards with the given power levels and heartbeat
 */
int setPowerLevels(uint8_t address, uint8_t coil1Level, uint8_t coil2Level, uint8_t heartbeat) {
  GeaCommandList cmd;

  if (!withinRange(coil1Level, minPowerSteps, maxPowerSteps) && !withinRange(coil2Level, minPowerSteps, maxPowerSteps)) {
    Serial.println("Power level out of range!");
    return -1;
  }

  int escapedBytes = 0;

  if (isEscaped(heartbeat)) {
    escapedBytes = 1;
  }

  typedef struct {
    GeaMessageHeader_t header;
    uint8_t command;
    uint8_t coil1Power;
    uint8_t coil2Power;
  } __attribute__((__packed__))SetPowerLevelsMsg_t;

  SetPowerLevelsMsg_t msg;
  cmd = CMD_SET_PWR_LEVELS;

  msg.header.sof = GEA_SOF;
  msg.header.destination = address;
  msg.header.length = 0x0b;
  msg.header.source = LOCAL_ADDR;
  msg.command = cmd;
  msg.coil1Power = coil1Level;
  msg.coil2Power = coil2Level;

  int txBufferSize = (sizeof(SetPowerLevelsMsg_t) + 5 + escapedBytes);
  char txBuffer[txBufferSize];

  memcpy(txBuffer, &msg, txBufferSize);

  if (escapedBytes > 0) {
    txBuffer[txBufferSize - 6] = GEA_ESC;
  }

  txBuffer[txBufferSize - 5] = heartbeat;
  uint16_t crc16 = CalculateCrc16(txBuffer, (txBufferSize + escapedBytes - 4));
  txBuffer[txBufferSize - 4] = crc16 & 0xff;
  txBuffer[txBufferSize - 3] = crc16 >> 8;
  txBuffer[txBufferSize - 2] = GEA_EOF;
  txBuffer[txBufferSize - 1] = GEA_ACK;

  for(int i=0; i<txBufferSize; i++) {
    Serial2.print(txBuffer[i]);
  }

  return 0;
}

/*
 * @brief Configures and initializes all generator boards depending on the personality
 */
int initCooktop(int personality) {
  PersonalityId personalityName = (PersonalityId)personality;
  
  switch(personalityName) {
    case PERSONALITY_FOUR_COILS:
      initSingleGenerator(GEN1_ADDR, COIL_TYPE_2500_WATT, COIL_TYPE_2500_WATT);
      delay(100);
      initSingleGenerator(GEN2_ADDR, COIL_TYPE_3700_WATT, COIL_TYPE_1800_WATT);
      delay(100);
      setPowerLevels(GEN1_ADDR, 0, 0, 0);
      delay(100);
      setPowerLevels(GEN2_ADDR, 0, 0, 0);
      delay(100);
      break;
    case PERSONALITY_FIVE_COILS:
      initSingleGenerator(GEN1_ADDR, COIL_TYPE_2500_WATT, COIL_TYPE_2500_WATT);
      delay(100);
      initSingleGenerator(GEN2_ADDR, COIL_TYPE_3700_WATT, COIL_TYPE_NONE);
      delay(100);
      initSingleGenerator(GEN3_ADDR, COIL_TYPE_1800_WATT, COIL_TYPE_3200_WATT);
      delay(100);
      setPowerLevels(GEN1_ADDR, 0, 0, 0);
      delay(100);
      setPowerLevels(GEN2_ADDR, 0, 0, 0);
      delay(100);
      setPowerLevels(GEN3_ADDR, 0, 0, 0);
      delay(100);
      break;
    default:
      break;
  }

  return 0;
}

/*
 * @brief Setup hardware interfaces and call init functions
 */
void setup() {
  Serial.begin(115200); // Console logging
  Serial2.begin(19200); // GEA2 bus

  Serial.println("Initializing potentiometers...");

  for (int i=0; i<5; i++) {
    pinMode(potPins[i], INPUT);
  }
  pinMode(personalitySelPin, INPUT);
  pinMode(heartbeatLed, OUTPUT);

  int personality;

  if (digitalRead(personalitySelPin) == 1) {
    personality = 1;
    numberOfCoils = 5;
  } else {
    personality = 0;
    numberOfCoils = 4;
  }
  
  Serial.print("Initializing generator boards for a ");
  if (personality == 1) {
    Serial.println("36 inch cooktop...");
  } else {
    Serial.println("30 inch cooktop...");
  }

  initCooktop(personality);
  Serial.println("Starting main loop...");
}

/*
 * Cyclically process pot values and set power levels, incrementing the heartbeat every time the levels are updated.
 */
void loop() {
  int potValues[5];
  int heartbeat;

  for(heartbeat = 0; heartbeat < 256; heartbeat++) {
    digitalWrite(heartbeatLed, !digitalRead(heartbeatLed));
    
    Serial.print("Power levels: ");

    for(int i=0; i<5; i++) {
      potValues[i] = map(analogRead(potPins[i]), 41, 4095, minPowerSteps, maxPowerSteps);
      Serial.print(potValues[i]);
      Serial.print(" ");
    }

    Serial.println();
    
    switch(numberOfCoils) {
      case 4:
        setPowerLevels(GEN1_ADDR, potValues[0], potValues[1], heartbeat);
        setPowerLevels(GEN2_ADDR, potValues[2], potValues[3], heartbeat);
        break;
      case 5:
        setPowerLevels(GEN1_ADDR, potValues[0], potValues[1], heartbeat);
        setPowerLevels(GEN2_ADDR, potValues[4], 0, heartbeat);
        setPowerLevels(GEN3_ADDR, potValues[2], potValues[3], heartbeat);
        break;
    }

    delay(500);
  }
}

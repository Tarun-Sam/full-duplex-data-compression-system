/*
  Hybrid Full-Duplex Arduino Compression System with OLED Display

  Arduino → Python:
    [0xAA] [mode] [hi] [lo] [data_len] [data...] [checksum] [0x55]

  Python → Arduino:
    [0xC2] [mode] [hi] [lo] [data_len] [data...] [checksum] [0x5A]

  mode:
    0x01 = Bit Packing
    0x02 = Zero-RLE
*/

#include <Wire.h>
#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define pot A0

#define Start 0xAA
#define End   0x55

#define REV_Start 0xC2
#define REV_End   0x5A

#define MODE_BITPACK 0x01
#define MODE_RLE     0x02

#define buffer_size 50
#define delta_count 49
#define packed_count 25
#define max_data_bytes 50

#define baud 115200
#define deadband 4

#define ADC_REF_VOLTAGE 5.0

int values[buffer_size];
int deltas[delta_count];

byte dataBytes[max_data_bytes];
byte dataLen = 0;
byte currentMode = MODE_RLE;

int laststablevalue = 0;
bool firstsample = true;

unsigned long lastsendtime = 0;
const unsigned long sendinterval = 500;

// -------------------- SETUP --------------------

void setup() {
  Serial.begin(baud);

  u8g2.begin();

  showStartupScreen();
}

// -------------------- MAIN LOOP --------------------

void loop() {
  if (millis() - lastsendtime >= sendinterval) {
    lastsendtime = millis();

    collectSamples();
    makeDeltas();

    currentMode = chooseCompressionMode();

    if (currentMode == MODE_RLE) {
      dataLen = compressZeroRLE();
    } else {
      dataLen = compressBitPacking();
    }

    sendCompressedPacket();

    // OLED is NOT updated here.
    // OLED should show only data returned from Python.
  }

  receiveCompressedPacket();
}

// -------------------- OLED DISPLAY FUNCTIONS --------------------

void showStartupScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.drawStr(0, 12, "Compression Project");
    u8g2.drawStr(0, 28, "OLED Ready");
    u8g2.drawStr(0, 44, "Waiting for Python");
    u8g2.drawStr(0, 60, "returned packet...");

  } while (u8g2.nextPage());
}

void showRxScreen(byte mode, int baseValue, byte len, int lastValue, float voltage) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.drawStr(0, 10, "Returned from Python");

    u8g2.setCursor(0, 22);
    u8g2.print("Mode: ");
    printModeName(mode);

    u8g2.setCursor(0, 34);
    u8g2.print("Base:");
    u8g2.print(baseValue);
    u8g2.print(" Len:");
    u8g2.print(len);

    u8g2.setCursor(0, 46);
    u8g2.print("ADC: ");
    u8g2.print(lastValue);

    u8g2.setCursor(0, 58);
    u8g2.print("Volt: ");
    u8g2.print(voltage, 2);
    u8g2.print(" V");

  } while (u8g2.nextPage());
}

void showErrorScreen(const char message[]) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.drawStr(0, 12, "Returned Packet Error");
    u8g2.drawStr(0, 30, message);

  } while (u8g2.nextPage());
}

void printModeName(byte mode) {
  if (mode == MODE_RLE) {
    u8g2.print("Zero-RLE");
  } else if (mode == MODE_BITPACK) {
    u8g2.print("Bit-Packing");
  } else {
    u8g2.print("Unknown");
  }
}

// -------------------- VOLTAGE CONVERSION --------------------

float adcToVoltage(int adcValue) {
  return adcValue * ADC_REF_VOLTAGE / 1023.0;
}

// -------------------- COLLECT DATA WITH DEADBAND --------------------

void collectSamples() {
  for (int i = 0; i < buffer_size; i++) {
    int raw = analogRead(pot);

    if (firstsample) {
      laststablevalue = raw;
      firstsample = false;
    }

    if (abs(raw - laststablevalue) >= deadband) {
      laststablevalue = raw;
    }

    values[i] = laststablevalue;
    delay(5);
  }
}

// -------------------- MAKE DELTAS --------------------

void makeDeltas() {
  for (int i = 1; i < buffer_size; i++) {
    int d = values[i] - values[i - 1];
    deltas[i - 1] = constrain(d, -8, 7);
  }
}

// -------------------- CHOOSE COMPRESSION MODE --------------------

byte chooseCompressionMode() {
  int zeroCount = 0;

  for (int i = 0; i < delta_count; i++) {
    if (deltas[i] == 0) {
      zeroCount++;
    }
  }

  float zeroRatio = (float)zeroCount / delta_count;

  if (zeroRatio >= 0.60) {
    return MODE_RLE;
  } else {
    return MODE_BITPACK;
  }
}

// -------------------- BIT PACKING COMPRESSION --------------------

byte compressBitPacking() {
  int outIndex = 0;

  for (int i = 0; i < delta_count; i += 2) {
    int d1 = deltas[i];
    int d2 = 0;

    if (i + 1 < delta_count) {
      d2 = deltas[i + 1];
    }

    byte n1 = deltaToNibble(d1);
    byte n2 = deltaToNibble(d2);

    dataBytes[outIndex++] = (n1 << 4) | n2;
  }

  return outIndex;
}

// -------------------- ZERO-RLE COMPRESSION --------------------

byte compressZeroRLE() {
  int outIndex = 0;
  int zeroRun = 0;

  for (int i = 0; i < delta_count; i++) {
    int d = deltas[i];

    if (d == 0) {
      zeroRun++;

      if (zeroRun == 127) {
        dataBytes[outIndex++] = 0x80 | zeroRun;
        zeroRun = 0;
      }
    } else {
      if (zeroRun > 0) {
        dataBytes[outIndex++] = 0x80 | zeroRun;
        zeroRun = 0;
      }

      dataBytes[outIndex++] = deltaToNibble(d);
    }
  }

  if (zeroRun > 0) {
    dataBytes[outIndex++] = 0x80 | zeroRun;
  }

  return outIndex;
}

// -------------------- SEND HYBRID COMPRESSED PACKET --------------------

void sendCompressedPacket() {
  byte hi = (values[0] >> 8) & 0xFF;
  byte lo = values[0] & 0xFF;

  byte checksum = 0;
  checksum ^= currentMode;
  checksum ^= hi;
  checksum ^= lo;
  checksum ^= dataLen;

  for (int i = 0; i < dataLen; i++) {
    checksum ^= dataBytes[i];
  }

  Serial.write(Start);
  Serial.write(currentMode);
  Serial.write(hi);
  Serial.write(lo);
  Serial.write(dataLen);

  for (int i = 0; i < dataLen; i++) {
    Serial.write(dataBytes[i]);
  }

  Serial.write(checksum);
  Serial.write(End);
}

// -------------------- RECEIVE COMPRESSED PACKET BACK --------------------

void receiveCompressedPacket() {
  if (Serial.available() < 1) {
    return;
  }

  if (Serial.peek() != REV_Start) {
    Serial.read();
    return;
  }

  Serial.read(); // remove REV_Start

  int mode = readByteTimeout();
  int hi = readByteTimeout();
  int lo = readByteTimeout();
  int len = readByteTimeout();

  if (mode < 0 || hi < 0 || lo < 0 || len < 0) {
    showErrorScreen("Header timeout");
    return;
  }

  if (len > max_data_bytes) {
    showErrorScreen("Length too large");
    return;
  }

  byte receivedData[max_data_bytes];

  for (int i = 0; i < len; i++) {
    int b = readByteTimeout();

    if (b < 0) {
      showErrorScreen("Data timeout");
      return;
    }

    receivedData[i] = (byte)b;
  }

  int receivedChecksum = readByteTimeout();
  int endByte = readByteTimeout();

  if (receivedChecksum < 0 || endByte < 0) {
    showErrorScreen("End timeout");
    return;
  }

  if (endByte != REV_End) {
    showErrorScreen("Wrong end byte");
    return;
  }

  byte calcChecksum = 0;
  calcChecksum ^= (byte)mode;
  calcChecksum ^= (byte)hi;
  calcChecksum ^= (byte)lo;
  calcChecksum ^= (byte)len;

  for (int i = 0; i < len; i++) {
    calcChecksum ^= receivedData[i];
  }

  if (calcChecksum != (byte)receivedChecksum) {
    showErrorScreen("Checksum failed");
    return;
  }

  int reconstructed[buffer_size];
  bool ok = false;

  if ((byte)mode == MODE_RLE) {
    ok = decompressZeroRLE((byte)hi, (byte)lo, receivedData, len, reconstructed);
  } else if ((byte)mode == MODE_BITPACK) {
    ok = decompressBitPacking((byte)hi, (byte)lo, receivedData, len, reconstructed);
  } else {
    showErrorScreen("Unknown mode");
    return;
  }

  if (!ok) {
    showErrorScreen("Decompress fail");
    return;
  }

  int baseValue = ((int)hi << 8) | lo;

  // This is the actual decompressed potentiometer ADC value.
  int lastValue = reconstructed[buffer_size - 1];

  // Convert ADC value to voltage.
  float voltage = adcToVoltage(lastValue);

  // OLED updates ONLY here, after valid returned packet from Python.
  showRxScreen((byte)mode, baseValue, (byte)len, lastValue, voltage);
}

// -------------------- ZERO-RLE DECOMPRESSION --------------------

bool decompressZeroRLE(byte hi, byte lo, byte receivedData[], int len, int reconstructed[]) {
  reconstructed[0] = ((int)hi << 8) | lo;

  int valueIndex = 1;

  for (int i = 0; i < len; i++) {
    byte token = receivedData[i];

    if (token & 0x80) {
      int runCount = token & 0x7F;

      for (int j = 0; j < runCount; j++) {
        if (valueIndex >= buffer_size) {
          return false;
        }

        reconstructed[valueIndex] = reconstructed[valueIndex - 1];
        valueIndex++;
      }
    } else {
      int delta = nibbleToDelta(token & 0x0F);

      if (valueIndex >= buffer_size) {
        return false;
      }

      reconstructed[valueIndex] = reconstructed[valueIndex - 1] + delta;
      valueIndex++;
    }
  }

  return valueIndex == buffer_size;
}

// -------------------- BIT PACKING DECOMPRESSION --------------------

bool decompressBitPacking(byte hi, byte lo, byte receivedData[], int len, int reconstructed[]) {
  reconstructed[0] = ((int)hi << 8) | lo;

  int valueIndex = 1;

  for (int i = 0; i < len; i++) {
    byte b = receivedData[i];

    byte upperNibble = (b >> 4) & 0x0F;
    byte lowerNibble = b & 0x0F;

    int delta1 = nibbleToDelta(upperNibble);
    int delta2 = nibbleToDelta(lowerNibble);

    if (valueIndex < buffer_size) {
      reconstructed[valueIndex] = reconstructed[valueIndex - 1] + delta1;
      valueIndex++;
    }

    if (valueIndex < buffer_size) {
      reconstructed[valueIndex] = reconstructed[valueIndex - 1] + delta2;
      valueIndex++;
    }
  }

  return valueIndex == buffer_size;
}

// -------------------- BYTE READ WITH TIMEOUT --------------------

int readByteTimeout() {
  unsigned long startTime = millis();

  while (Serial.available() == 0) {
    if (millis() - startTime > 50) {
      return -1;
    }
  }

  return Serial.read();
}

// -------------------- 4-BIT SIGNED CONVERSION --------------------

byte deltaToNibble(int delta) {
  if (delta < 0) {
    return (byte)(delta + 16);
  }

  return (byte)delta;
}

int nibbleToDelta(byte nibble) {
  if (nibble >= 8) {
    return nibble - 16;
  }

  return nibble;
}

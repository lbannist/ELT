//
// Created by indrek on 1.05.2016.
//

// set EXAMPLE to EXAMPLE_UART in setup.h to activate
#include "setup.h"
#if EXAMPLE == 3
#include "Arduino.h"
#include "CameraOV7670.h"


// select resolution and communication speed:
//  1 - 115200bps 160x120 rgb
//  2 - 115200bps 160x120 grayscale
//  3 - 500000bps 160x120 rgb
//  4 - 500000bps 160x120 grayscale
//  5 - 500000bps 320x240 rgb
//  6 - 500000bps 320x240 grayscale
//  7 - 1Mbps 160x120 rgb
//  8 - 1Mbps 160x120 grayscale
//  9 - 1Mbps 320x240 rgb
// 10 - 1Mbps 320x240 grayscale
// 11 - 1Mbps 640x480 grayscale
// 12 - 2Mbps 160x120 rgb
// 13 - 2Mbps 160x120 grayscale
// 14 - 2Mbps 320x240 rgb
// 15 - 2Mbps 320x240 grayscale
// 16 - 2Mbps 640x480 rgb
// 17 - 2Mbps 640x480 grayscale
#define UART_MODE 5




const uint8_t VERSION = 0x10;
const uint8_t COMMAND_NEW_FRAME = 0x01 | VERSION;
const uint8_t COMMAND_DEBUG_DATA = 0x03 | VERSION;

const uint16_t UART_PIXEL_FORMAT_RGB565 = 0x01;
const uint16_t UART_PIXEL_FORMAT_GRAYSCALE = 0x02;

// Pixel byte parity check:
// Pixel Byte H: odd number of bits under H_BYTE_PARITY_CHECK and H_BYTE_PARITY_INVERT
// Pixel Byte L: even number of bits under L_BYTE_PARITY_CHECK and L_BYTE_PARITY_INVERT
//                                          H:RRRRRGGG
const uint8_t H_BYTE_PARITY_CHECK =  0b00100000;
const uint8_t H_BYTE_PARITY_INVERT = 0b00001000;
//                                          L:GGGBBBBB
const uint8_t L_BYTE_PARITY_CHECK =  0b00001000;
const uint8_t L_BYTE_PARITY_INVERT = 0b00100000;
// Since the parity for L byte can be zero we must ensure that the total byet value is above zero.
// Increasing the lowest bit of blue color is OK for that.
const uint8_t L_BYTE_PREVENT_ZERO  = 0b00000001;


const uint16_t COLOR_GREEN = 0x07E0;
const uint16_t COLOR_RED = 0xF800;

// ============================================================================
// STOPLIGHT CHANGE 1 of 5: new colour, new command and constants
// (everything marked "STOPLIGHT CHANGE" is new; the rest is the original example)
// ============================================================================
const uint16_t COLOR_YELLOW = 0xFFE0;

// ---- Stoplight start-up checks (classroom edition) -------------------------
// The PC program (arduimagecapture.py) turns these into the red/yellow/green
// stoplight display. COMMAND_CHECKPOINT payload: stage, result, extra bytes.
const uint8_t COMMAND_CHECKPOINT = 0x04 | VERSION;
const uint8_t STAGE_RED = 1;      // Arduino runs this sketch and talks to the PC
const uint8_t STAGE_YELLOW = 2;   // Camera is powered and answers on I2C
const uint8_t STAGE_GREEN = 3;    // Camera clock, sync and all 8 data wires work
const uint8_t STOPLIGHT_SKETCH_VERSION = 1;   // the PC program checks this number
const uint8_t RESULT_FAIL = 0;
const uint8_t RESULT_PASS = 1;
const uint8_t RESULT_TESTING = 2;

const uint8_t OV7670_I2C_ADDRESS = 0x21;
const uint8_t OV7670_REG_PID = 0x0A;      // should read 0x76
const uint8_t OV7670_REG_VER = 0x0B;      // should read 0x73
const uint16_t BLANK_WIDTH = 40;          // checkpoint pictures are small so they send quickly
const uint16_t BLANK_HEIGHT = 30;
const uint16_t DATA_TEST_MS = 3000;       // how long the data-wire test listens
const uint16_t MIN_PIXEL_CLOCK_EDGES = 64;



void processGrayscaleFrameBuffered();
void processGrayscaleFrameDirect();
void processRgbFrameBuffered();
void processRgbFrameDirect();
typedef void (*ProcessFrameData)(void) ;


#if UART_MODE==1
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 115200;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_RGB565, 34);
#endif

#if UART_MODE==2
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 115200;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_YUV422, 17);
#endif

#if UART_MODE==3
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 500000;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_RGB565, 8);
#endif

#if UART_MODE==4
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 500000;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_YUV422, 4);
#endif

#if UART_MODE==5
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 500000;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_RGB565, 32);
#endif

#if UART_MODE==6
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 500000;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_YUV422, 16);
#endif

#if UART_MODE==7
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 1000000;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = false;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_RGB565, 5);
#endif

#if UART_MODE==8
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 1000000;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = false;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_YUV422, 2);
#endif

#if UART_MODE==9
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 1000000;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_RGB565, 16);
#endif

#if UART_MODE==10
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 1000000;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_YUV422, 8);
#endif

#if UART_MODE==11
const uint16_t lineLength = 640;
const uint16_t lineCount = 480;
const uint32_t baud  = 1000000;
const ProcessFrameData processFrameData = processGrayscaleFrameDirect;
const uint16_t lineBufferLength = 1;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_VGA_640x480, CameraOV7670::PIXEL_YUV422, 39);
#endif

#if UART_MODE==12
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 2000000;
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = false;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_RGB565, 2);
#endif

#if UART_MODE==13
const uint16_t lineLength = 160;
const uint16_t lineCount = 120;
const uint32_t baud  = 2000000;
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = false;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QQVGA_160x120, CameraOV7670::PIXEL_YUV422, 2);
#endif

#if UART_MODE==14
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 2000000; // may be unreliable
const ProcessFrameData processFrameData = processRgbFrameBuffered;
const uint16_t lineBufferLength = lineLength * 2;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_RGB565, 12);
#endif

#if UART_MODE==15
const uint16_t lineLength = 320;
const uint16_t lineCount = 240;
const uint32_t baud  = 2000000; // may be unreliable
const ProcessFrameData processFrameData = processGrayscaleFrameBuffered;
const uint16_t lineBufferLength = lineLength;
const bool isSendWhileBuffering = false;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_YUV422, 6);
#endif

#if UART_MODE==16
const uint16_t lineLength = 640;
const uint16_t lineCount = 480;
const uint32_t baud  = 2000000;
const ProcessFrameData processFrameData = processRgbFrameDirect;
const uint16_t lineBufferLength = 1;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_RGB565;
CameraOV7670 camera(CameraOV7670::RESOLUTION_VGA_640x480, CameraOV7670::PIXEL_RGB565, 39);
#endif

#if UART_MODE==17
const uint16_t lineLength = 640;
const uint16_t lineCount = 480;
const uint32_t baud  = 2000000;
const ProcessFrameData processFrameData = processGrayscaleFrameDirect;
const uint16_t lineBufferLength = 1;
const bool isSendWhileBuffering = true;
const uint8_t uartPixelFormat = UART_PIXEL_FORMAT_GRAYSCALE;
CameraOV7670 camera(CameraOV7670::RESOLUTION_VGA_640x480, CameraOV7670::PIXEL_YUV422, 19);
#endif


uint8_t lineBuffer [lineBufferLength]; // Two bytes per pixel
uint8_t * lineBufferSendByte;
bool isLineBufferSendHighByte;
bool isLineBufferByteFormatted;

uint16_t frameCounter = 0;
uint16_t processedByteCountDuringCameraRead = 0;


void commandStartNewFrame(uint8_t pixelFormat);
// STOPLIGHT CHANGE 2 of 5: prototypes for the new functions
void commandStartNewFrameSized(uint16_t width, uint16_t height, uint8_t pixelFormat);
void sendCheckpoint(uint8_t stage, uint8_t result, const uint8_t * extra, uint8_t extraLength);
void commandDebugPrint(const String debugText);
uint8_t sendNextCommandByte(uint8_t checksum, uint8_t commandByte);

void sendBlankFrame(uint16_t color);
void runStageYellow();
void runStageGreen();
void sendStageOneReport();
inline void processNextGrayscalePixelByteInBuffer() __attribute__((always_inline));
inline void processNextRgbPixelByteInBuffer() __attribute__((always_inline));
inline void tryToSendNextRgbPixelByteInBuffer() __attribute__((always_inline));
inline void formatNextRgbPixelByteInBuffer() __attribute__((always_inline));
inline uint8_t formatRgbPixelByteH(uint8_t byte) __attribute__((always_inline));
inline uint8_t formatRgbPixelByteL(uint8_t byte) __attribute__((always_inline));
inline uint8_t formatPixelByteGrayscaleFirst(uint8_t byte) __attribute__((always_inline));
inline uint8_t formatPixelByteGrayscaleSecond(uint8_t byte) __attribute__((always_inline));
inline void waitForPreviousUartByteToBeSent() __attribute__((always_inline));
inline bool isUartReady() __attribute__((always_inline));



// ============================================================================
// STOPLIGHT CHANGE 3 of 5: start-up checks (stage 1 RED, 2 YELLOW, 3 GREEN)
// replaces the original initializeScreenAndCamera() and sendBlankFrame()
// ============================================================================

// Camera used ONLY for the data-wire test. Same settings as UART_MODE 9
// (known good: RGB565, QVGA, moderate pixel clock). The real camera is set up
// afterwards with the settings of the selected UART_MODE.
CameraOV7670 testCamera(CameraOV7670::RESOLUTION_QVGA_320x240, CameraOV7670::PIXEL_RGB565, 16);

struct DataLineReport {
  uint8_t seenHigh;   // bit n set = data line Dn was seen HIGH at least once
  uint8_t seenLow;    // bit n set = data line Dn was seen LOW at least once
  uint8_t flags;      // 0x01 VSYNC high seen, 0x02 VSYNC low seen,
                      // 0x04 PCLK high seen,  0x08 PCLK low seen,
                      // 0x10 enough PCLK edges, 0x20 camera stopped answering
};


// this is called in Arduino setup() function
void initializeScreenAndCamera() {

  // Enable this for WAVGAT CPUs
  // For UART communiation we want to set WAVGAT Nano to 16Mhz to match Atmel based Arduino
  //CLKPR = 0x80; // enter clock rate change mode
  //CLKPR = 1; // set prescaler to 1. WAVGAT MCU has it 3 by default.

  Serial.begin(baud);

  // Stage 1 - RED: if the PC can read this, the sketch, libraries and baud rate are fine.
  // The message also tells the PC which sketch, mode, board and memory it is dealing with.
  sendStageOneReport();
  sendBlankFrame(COLOR_RED);
  delay(1200);

  do {
    runStageYellow();   // waits (and retests) until the camera answers
    runStageGreen();    // waits (and retests) until clock, sync and data wires are good
  } while (!camera.init());   // final camera setup for the selected UART_MODE
}


extern int __heap_start, *__brkval;

uint16_t getFreeRam() {
  int marker;
  return (uint16_t)((int)&marker - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
}


// Stage 1 facts for the PC program (13 bytes, same order the PC program reads them).
void sendStageOneReport() {
  uint8_t board = 0;   // 1 = Uno/Nano (ATmega328P/168), 2 = Mega, 0 = something else
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  board = 1;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  board = 2;
#endif
  uint16_t baudDiv100 = baud / 100;
  uint16_t freeRam = getFreeRam();
  uint8_t extra[13] = {
    STOPLIGHT_SKETCH_VERSION,
    UART_MODE,
    (uint8_t)(lineLength & 0xFF), (uint8_t)(lineLength >> 8),
    (uint8_t)(lineCount & 0xFF), (uint8_t)(lineCount >> 8),
    (uint8_t)uartPixelFormat,
    (uint8_t)(baudDiv100 & 0xFF), (uint8_t)(baudDiv100 >> 8),
    (uint8_t)(F_CPU / 1000000UL),
    board,
    (uint8_t)(freeRam & 0xFF), (uint8_t)(freeRam >> 8)
  };
  sendCheckpoint(STAGE_RED, RESULT_PASS, extra, sizeof(extra));
}


// ---- Stage 2 helper: check the two camera I2C wires one by one ---------------
// Returns flags: 0x01 = SIOD (A4) rises by itself (10K pull-up to 3.3V present)
//                0x02 = SIOC (A5) rises by itself
//                0x04 = SIOD and SIOC are shorted together
// Must be called while Wire is switched off (Wire.end()).
const uint8_t I2C_SDA_PIN = A4;
const uint8_t I2C_SCL_PIN = A5;

bool lineRisesByItself(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);       // pull the line low...
  delayMicroseconds(30);
  pinMode(pin, INPUT);          // ...let go (Arduino pull-up OFF)...
  delayMicroseconds(30);
  return digitalRead(pin) == HIGH;   // ...a 10K resistor to 3.3V brings it back up quickly
}

bool pullLowMakesOtherLow(uint8_t pullPin, uint8_t otherPin) {
  pinMode(otherPin, INPUT);
  pinMode(pullPin, OUTPUT);
  digitalWrite(pullPin, LOW);
  delayMicroseconds(30);
  bool otherIsLow = digitalRead(otherPin) == LOW;
  pinMode(pullPin, INPUT);
  return otherIsLow;
}

uint8_t testI2cLines() {
  uint8_t flags = 0;
  if (lineRisesByItself(I2C_SDA_PIN)) flags |= 0x01;
  if (lineRisesByItself(I2C_SCL_PIN)) flags |= 0x02;
  if ((flags & 0x03) == 0x03 && pullLowMakesOtherLow(I2C_SDA_PIN, I2C_SCL_PIN)) flags |= 0x04;
  return flags;
}


// Returns 0 when OK, otherwise the Wire error code (2 = no answer, 5 = timeout) or 6 = no data.
uint8_t readCameraRegisterChecked(uint8_t reg, uint8_t & value) {
  Wire.beginTransmission(OV7670_I2C_ADDRESS);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    return error;
  }
  if (Wire.requestFrom(OV7670_I2C_ADDRESS, (uint8_t)1) != 1) {
    return 6;
  }
  value = Wire.read();
  return 0;
}


// Stage 2 - YELLOW: the camera is powered and answers on I2C (3.3V, GND, SIOD, SIOC).
void runStageYellow() {
  while (true) {
    OV7670_INIT_CLOCK_OUT;   // start the camera clock first, like camera.init() does
    Wire.end();              // I2C off, so the two wires can be tested one by one
    uint8_t lineFlags = testI2cLines();
    Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
    Wire.setWireTimeout(25000, true);   // never hang on a bad SIOD/SIOC wire
#endif
    delay(20);

    uint8_t pid = 0;
    uint8_t ver = 0;
    uint8_t error = readCameraRegisterChecked(OV7670_REG_PID, pid);
    if (error == 0) {
      error = readCameraRegisterChecked(OV7670_REG_VER, ver);
    }
    bool isOk = (error == 0) && (pid == 0x76);

    uint8_t extra[4] = {error, pid, ver, lineFlags};
    sendCheckpoint(STAGE_YELLOW, isOk ? RESULT_PASS : RESULT_FAIL, extra, 4);
    if (isOk) {
      break;
    }
    delay(1500);   // retest, so a fixed wire turns the stage yellow without a restart
  }
  sendBlankFrame(COLOR_YELLOW);
  delay(1200);
}


void setDataTestPullups(bool enable) {
  // With pull-ups on, a data wire that is not connected reads a steady HIGH,
  // while a connected camera output overrides the weak pull-up.
  const uint8_t pins[] = {2, 12, 4, 5, 6, 7, A0, A1, A2, A3};
  for (uint8_t i = 0; i < sizeof(pins); i++) {
    pinMode(pins[i], enable ? INPUT_PULLUP : INPUT);
  }
}


DataLineReport testDataLines() {
  DataLineReport report = {0, 0, 0};

  // Camera shows its built-in colour bars (white ... black), so every data bit
  // must switch between 0 and 1 many times per line.
  if (!testCamera.init()) {
    report.flags |= 0x20;
  }
  testCamera.showColorBars(false);

  setDataTestPullups(true);
  uint16_t edges = 0;
  uint8_t lastClock = OV7670_PIXEL_CLOCK;
  uint16_t loops = 0;
  uint32_t startMs = millis();

  while (true) {
    if ((++loops & 0x03FF) == 0 && (millis() - startMs) > DATA_TEST_MS) {
      break;
    }
    uint8_t clock = OV7670_PIXEL_CLOCK;
    if (clock && !lastClock) {
      // Rising edge of the pixel clock: read the data lines right away.
      uint8_t pixelByte;
      OV7670_READ_PIXEL_BYTE(pixelByte);
      report.seenHigh |= pixelByte;
      report.seenLow |= (uint8_t)~pixelByte;
      if (edges < 0xFFFF) {
        edges++;
      }
    }
    lastClock = clock;
    report.flags |= clock ? 0x04 : 0x08;
    report.flags |= OV7670_VSYNC ? 0x01 : 0x02;
  }
  setDataTestPullups(false);

  if (edges >= MIN_PIXEL_CLOCK_EDGES) {
    report.flags |= 0x10;
  }
  return report;
}


// Stage 3 - GREEN: camera clock + VSYNC + PCLK + all 8 data wires work.
void runStageGreen() {
  while (true) {
    sendCheckpoint(STAGE_GREEN, RESULT_TESTING, 0, 0);
    DataLineReport report = testDataLines();

    bool isOk = (report.flags & 0x30) == 0x10
                && (report.flags & 0x03) == 0x03
                && report.seenHigh == 0xFF
                && report.seenLow == 0xFF;

    uint8_t extra[3] = {report.seenHigh, report.seenLow, report.flags};
    sendCheckpoint(STAGE_GREEN, isOk ? RESULT_PASS : RESULT_FAIL, extra, 3);
    if (isOk) {
      break;
    }
    delay(1500);
  }
  sendBlankFrame(COLOR_GREEN);
  delay(1200);
}


void sendBlankFrame(uint16_t color) {
  uint8_t colorH = (color >> 8) & 0xFF;
  uint8_t colorL = color & 0xFF;

  commandStartNewFrameSized(BLANK_WIDTH, BLANK_HEIGHT, UART_PIXEL_FORMAT_RGB565);
  for (uint16_t j=0; j<BLANK_HEIGHT; j++) {
    for (uint16_t i=0; i<BLANK_WIDTH; i++) {
      waitForPreviousUartByteToBeSent();
      UDR0 = formatRgbPixelByteH(colorH);
      waitForPreviousUartByteToBeSent();
      UDR0 = formatRgbPixelByteL(colorL);
    }
  }
}




// this is called in Arduino loop() function
void processFrame() {
  processedByteCountDuringCameraRead = 0;
  commandStartNewFrame(uartPixelFormat);
  noInterrupts();
  processFrameData();
  interrupts();
  frameCounter++;
  commandDebugPrint("Frame " + String(frameCounter)/* + " " + String(processedByteCountDuringCameraRead)*/);
  //commandDebugPrint("Frame " + String(frameCounter, 16)); // send number in hexadecimal
}


void processGrayscaleFrameBuffered() {
  camera.waitForVsync();
  commandDebugPrint("Vsync");

  camera.ignoreVerticalPadding();

  for (uint16_t y = 0; y < lineCount; y++) {
    lineBufferSendByte = &lineBuffer[0];
    camera.ignoreHorizontalPaddingLeft();

    uint16_t x = 0;
    while ( x < lineBufferLength) {
      camera.waitForPixelClockRisingEdge(); // YUV422 grayscale byte
      camera.readPixelByte(lineBuffer[x]);
      lineBuffer[x] = formatPixelByteGrayscaleFirst(lineBuffer[x]);

      camera.waitForPixelClockRisingEdge(); // YUV422 color byte. Ignore.
      if (isSendWhileBuffering) {
        processNextGrayscalePixelByteInBuffer();
      }
      x++;

      camera.waitForPixelClockRisingEdge(); // YUV422 grayscale byte
      camera.readPixelByte(lineBuffer[x]);
      lineBuffer[x] = formatPixelByteGrayscaleSecond(lineBuffer[x]);

      camera.waitForPixelClockRisingEdge(); // YUV422 color byte. Ignore.
      if (isSendWhileBuffering) {
        processNextGrayscalePixelByteInBuffer();
      }
      x++;
    }
    camera.ignoreHorizontalPaddingRight();

    // Debug info to get some feedback how mutch data was processed during line read.
    processedByteCountDuringCameraRead = lineBufferSendByte - (&lineBuffer[0]);

    // Send rest of the line
    while (lineBufferSendByte < &lineBuffer[lineLength]) {
      processNextGrayscalePixelByteInBuffer();
    }
  };
}

void processNextGrayscalePixelByteInBuffer() {
  if (isUartReady()) {
    UDR0 = *lineBufferSendByte;
    lineBufferSendByte++;
  }
}


void processGrayscaleFrameDirect() {
  camera.waitForVsync();
  commandDebugPrint("Vsync");

  camera.ignoreVerticalPadding();

  for (uint16_t y = 0; y < lineCount; y++) {
    camera.ignoreHorizontalPaddingLeft();

    uint16_t x = 0;
    while ( x < lineLength) {
      camera.waitForPixelClockRisingEdge(); // YUV422 grayscale byte
      camera.readPixelByte(lineBuffer[0]);
      lineBuffer[0] = formatPixelByteGrayscaleFirst(lineBuffer[0]);

      camera.waitForPixelClockRisingEdge(); // YUV422 color byte. Ignore.
      waitForPreviousUartByteToBeSent();
      UDR0 = lineBuffer[0];
      x++;

      camera.waitForPixelClockRisingEdge(); // YUV422 grayscale byte
      camera.readPixelByte(lineBuffer[0]);
      lineBuffer[0] = formatPixelByteGrayscaleSecond(lineBuffer[0]);

      camera.waitForPixelClockRisingEdge(); // YUV422 color byte. Ignore.
      waitForPreviousUartByteToBeSent();
      UDR0 = lineBuffer[0];
      x++;
    }

    camera.ignoreHorizontalPaddingRight();
  }
}

uint8_t formatPixelByteGrayscaleFirst(uint8_t pixelByte) {
  // For the First byte in the parity chek byte pair the last bit is always 0.
  pixelByte &= 0b11111110;
  if (pixelByte == 0) {
    // Make pixel color always slightly above 0 since zero is a command marker.
    pixelByte |= 0b00000010;
  }
  return pixelByte;
}

uint8_t formatPixelByteGrayscaleSecond(uint8_t pixelByte) {
  // For the second byte in the parity chek byte pair the last bit is always 1.
  return pixelByte | 0b00000001;
}



void processRgbFrameBuffered() {
  camera.waitForVsync();
  commandDebugPrint("Vsync");

  camera.ignoreVerticalPadding();

  for (uint16_t y = 0; y < lineCount; y++) {
    lineBufferSendByte = &lineBuffer[0];
    isLineBufferSendHighByte = true; // Line starts with High byte
    isLineBufferByteFormatted = false;

    camera.ignoreHorizontalPaddingLeft();

    for (uint16_t x = 0; x < lineBufferLength;  x++) {
      camera.waitForPixelClockRisingEdge();
      camera.readPixelByte(lineBuffer[x]);
      if (isSendWhileBuffering) {
        processNextRgbPixelByteInBuffer();
      }
    };

    camera.ignoreHorizontalPaddingRight();

    // Debug info to get some feedback how mutch data was processed during line read.
    processedByteCountDuringCameraRead = lineBufferSendByte - (&lineBuffer[0]);

    // send rest of the line
    while (lineBufferSendByte < &lineBuffer[lineLength * 2]) {
      processNextRgbPixelByteInBuffer();
    }
  }
}

void processNextRgbPixelByteInBuffer() {
  // Format pixel bytes and send out in different cycles.
  // There is not enough time to do both on faster frame rates.
  if (isLineBufferByteFormatted) {
    tryToSendNextRgbPixelByteInBuffer();
  } else {
    formatNextRgbPixelByteInBuffer();
  }
}

void tryToSendNextRgbPixelByteInBuffer() {
  if (isUartReady()) {
    UDR0 = *lineBufferSendByte;
    lineBufferSendByte++;
    isLineBufferByteFormatted = false;
  }
}

void formatNextRgbPixelByteInBuffer() {
  if (isLineBufferSendHighByte) {
    *lineBufferSendByte = formatRgbPixelByteH(*lineBufferSendByte);
  } else {
    *lineBufferSendByte = formatRgbPixelByteL(*lineBufferSendByte);
  }
  isLineBufferByteFormatted = true;
  isLineBufferSendHighByte = !isLineBufferSendHighByte;
}




void processRgbFrameDirect() {
  camera.waitForVsync();
  commandDebugPrint("Vsync");

  camera.ignoreVerticalPadding();

  for (uint16_t y = 0; y < lineCount; y++) {
    camera.ignoreHorizontalPaddingLeft();
    
    for (uint16_t x = 0; x < lineLength; x++) {
      
      camera.waitForPixelClockRisingEdge();
      camera.readPixelByte(lineBuffer[0]);
      lineBuffer[0] = formatRgbPixelByteH(lineBuffer[0]);
      waitForPreviousUartByteToBeSent();
      UDR0 = lineBuffer[0];
      
      camera.waitForPixelClockRisingEdge();
      camera.readPixelByte(lineBuffer[0]);
      lineBuffer[0] = formatRgbPixelByteL(lineBuffer[0]);
      waitForPreviousUartByteToBeSent();
      UDR0 = lineBuffer[0];
    }
    
    camera.ignoreHorizontalPaddingRight();
  };
}


// RRRRRGGG
uint8_t formatRgbPixelByteH(uint8_t pixelByteH) {
  // Make sure that
  // A: pixel color always slightly above 0 since zero is end of line marker
  // B: odd number of bits for H byte under H_BYTE_PARITY_CHECK and H_BYTE_PARITY_INVERT to enable error correction
  if (pixelByteH & H_BYTE_PARITY_CHECK) {
    return pixelByteH & (~H_BYTE_PARITY_INVERT);
  } else {
    return pixelByteH | H_BYTE_PARITY_INVERT;
  }
}


// GGGBBBBB
uint8_t formatRgbPixelByteL(uint8_t pixelByteL) {
  // Make sure that
  // A: pixel color always slightly above 0 since zero is end of line marker
  // B: even number of bits for L byte under L_BYTE_PARITY_CHECK and L_BYTE_PARITY_INVERT to enable error correction
  if (pixelByteL & L_BYTE_PARITY_CHECK) {
    return pixelByteL | L_BYTE_PARITY_INVERT | L_BYTE_PREVENT_ZERO;
  } else {
    return (pixelByteL & (~L_BYTE_PARITY_INVERT)) | L_BYTE_PREVENT_ZERO;
  }
}









// ============================================================================
// STOPLIGHT CHANGE 4 of 5: commandStartNewFrame() now calls a version that
// accepts any picture size (the checkpoint pictures are small, 40 x 30)
// ============================================================================
void commandStartNewFrame(uint8_t pixelFormat) {
  commandStartNewFrameSized(lineLength, lineCount, pixelFormat);
}


void commandStartNewFrameSized(uint16_t width, uint16_t height, uint8_t pixelFormat) {
  waitForPreviousUartByteToBeSent();
  UDR0 = 0x00; // New command

  waitForPreviousUartByteToBeSent();
  UDR0 = 4; // Command length

  uint8_t checksum = 0;
  checksum = sendNextCommandByte(checksum, COMMAND_NEW_FRAME);
  checksum = sendNextCommandByte(checksum, width & 0xFF); // lower 8 bits of image width
  checksum = sendNextCommandByte(checksum, height & 0xFF); // lower 8 bits of image height
  checksum = sendNextCommandByte(checksum,
      ((width >> 8) & 0x03) // higher 2 bits of image width
      | ((height >> 6) & 0x0C) // higher 2 bits of image height
      | ((pixelFormat << 4) & 0xF0));

  waitForPreviousUartByteToBeSent();
  UDR0 = checksum;
}


// ============================================================================
// STOPLIGHT CHANGE 5 of 5: the new "checkpoint" message the PC program shows
// as the stoplight and the checklists
// ============================================================================
void sendCheckpoint(uint8_t stage, uint8_t result, const uint8_t * extra, uint8_t extraLength) {
  waitForPreviousUartByteToBeSent();
  UDR0 = 0x00; // New command

  waitForPreviousUartByteToBeSent();
  UDR0 = 3 + extraLength; // Command length: command code + stage + result + extra bytes

  uint8_t checksum = 0;
  checksum = sendNextCommandByte(checksum, COMMAND_CHECKPOINT);
  checksum = sendNextCommandByte(checksum, stage);
  checksum = sendNextCommandByte(checksum, result);
  for (uint8_t i = 0; i < extraLength; i++) {
    checksum = sendNextCommandByte(checksum, extra[i]);
  }

  waitForPreviousUartByteToBeSent();
  UDR0 = checksum;
}


void commandDebugPrint(const String debugText) {
  if (debugText.length() > 0) {
    
    waitForPreviousUartByteToBeSent();
    UDR0 = 0x00; // New commnad

    waitForPreviousUartByteToBeSent();
    UDR0 = debugText.length() + 1; // Command length. +1 for command code.
    
    uint8_t checksum = 0;
    checksum = sendNextCommandByte(checksum, COMMAND_DEBUG_DATA);
    for (uint16_t i=0; i<debugText.length(); i++) {
      checksum = sendNextCommandByte(checksum, debugText[i]);
    }

    waitForPreviousUartByteToBeSent();
    UDR0 = checksum;
  }
}


uint8_t sendNextCommandByte(uint8_t checksum, uint8_t commandByte) {
  waitForPreviousUartByteToBeSent();
  UDR0 = commandByte;
  return checksum ^ commandByte;
}




void waitForPreviousUartByteToBeSent() {
  while(!isUartReady()); //wait for byte to transmit
}


bool isUartReady() {
  return UCSR0A & (1<<UDRE0);
}


#endif

#include <Arduino.h>
#include <math.h>

#define SEQUENCE_LENGTH 1024

unsigned int STEP_DELAY = 4000;
unsigned int currentX = 0;
unsigned int currentY = 0;

const int LDirPin = 2;
const int LDirInit = 1;
const int LStepPin = 3;

const int RDirPin = 4;
const int RDirInit = -1;
const int RStepPin = 5;

// State definitions
enum State {DISCONNECTED, READY_FOR_NEW_DRAWING, AWAITING_COMMAND_SEQUENCE, READING_COMMAND_SEQUENCE, DRAWING};

// Function prototypes
void move_to_initial_position(int x, int y);
void draw_binary_data(byte *data, int length);

// Global variables
State current_state = DISCONNECTED;
byte buffer[SEQUENCE_LENGTH * 4]; // Assuming a predefined max size of 256 bytes
char strBuffer[1024];

void setup() {
  Serial.begin(9600);

  pinMode(LStepPin, OUTPUT);
	pinMode(LDirPin, OUTPUT);
  pinMode(RStepPin, OUTPUT);
	pinMode(RDirPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("CONNECT")) {
      current_state = READY_FOR_NEW_DRAWING;
      sprintf(strBuffer, "CONFIG:sequence_length=%d", SEQUENCE_LENGTH);
      Serial.println(strBuffer);
      Serial.println("STATE:READY_FOR_NEW_DRAWING");
    } else if (command.startsWith("INIT:")) {
      if (current_state == READY_FOR_NEW_DRAWING) {
        int commaIndex = command.indexOf(',');
        int x = command.substring(5, commaIndex).toInt();
        int y = command.substring(commaIndex + 1).toInt();

        current_state = DRAWING;
        Serial.println("STATE:DRAWING");
        move_to_initial_position(x, y);
        current_state = AWAITING_COMMAND_SEQUENCE;
        Serial.println("STATE:AWAITING_COMMAND_SEQUENCE");
      } else {
        Serial.println("ERROR:Init called in wrong state");
      }
    } else if (command.startsWith("DRAW:")) {
      if (current_state == AWAITING_COMMAND_SEQUENCE) {
        int num_bytes = command.substring(5).toInt();
        if (num_bytes > SEQUENCE_LENGTH * 4) {
          Serial.println("ERROR:Too many input bytes");
        }
        current_state = READING_COMMAND_SEQUENCE;
        Serial.println("STATE:READING_COMMAND_SEQUENCE");
        int length = Serial.readBytes(buffer, num_bytes);

        current_state = DRAWING;
        Serial.println("STATE:DRAWING");
        unsigned int numControlPoints = length / 4;
        draw_binary_data(buffer, numControlPoints);
        current_state = AWAITING_COMMAND_SEQUENCE;
        Serial.println("STATE:AWAITING_COMMAND_SEQUENCE");
      } else {
        Serial.println("ERROR:Draw called in wrong state");
      }
    } else if (command.startsWith("END_DRAWING")) {
      if (current_state == AWAITING_COMMAND_SEQUENCE) {
        current_state = READY_FOR_NEW_DRAWING;
        Serial.println("STATE:READY_FOR_NEW_DRAWING");
      } else {
        Serial.println("ERROR:End drawing called in wrong state");
      }
    } else {
      Serial.println("ERROR:Unrecognized command");
    }
  }
}

void executeDelta(int deltaX, int deltaY) {
  unsigned int xSteps = abs(deltaX);
  unsigned int ySteps = abs(deltaY);
  int xDir = deltaX > 0 ? 1 : -1;
  int yDir = deltaY > 0 ? 1 : -1;

  // Serial.print("Deltas: ");
  // Serial.print(deltaX);
  // Serial.print(", ");
  // Serial.println(deltaY);

  // Now we need to plan the series of steps that will execute this path
  bool xIsMax = xSteps > ySteps;
  unsigned int mainSteps = xIsMax ? xSteps : ySteps;
  unsigned int offSteps = xIsMax ? ySteps : xSteps;
  int mainDir = xIsMax ? xDir : yDir;
  int offDir = xIsMax ? yDir : xDir;

  int mainStepPin = xIsMax ? LStepPin : RStepPin;
  int offStepPin = xIsMax ? RStepPin : LStepPin;
  int mainDirPin = xIsMax ? LDirPin : RDirPin;
  int offDirPin = xIsMax ? RDirPin : LDirPin;
  int mainDirInit = xIsMax ? LDirInit : RDirInit;
  int offDirInit = xIsMax ? RDirInit : LDirInit;

  digitalWrite(mainDirPin, mainDir == mainDirInit ? HIGH : LOW);
  digitalWrite(offDirPin, offDir == offDirInit ? HIGH : LOW);

  float offStepsPerMainStep = offSteps > 0 ? (float) offSteps / (float) mainSteps : -1.0;

  unsigned int executedOffSteps = 0;
  for (int executedMainSteps = 0; executedMainSteps < mainSteps; executedMainSteps++) {
    // On every loop we execute a main step. If i % mainStepsPerOffStep == 0 then we also execute an off step
    // Unless mainStepsPerOffStep == -1 in which case we never do an off step.
    digitalWrite(mainStepPin, HIGH);
    if (offStepsPerMainStep > 0 && offStepsPerMainStep * executedMainSteps > executedOffSteps) {
      digitalWrite(offStepPin, HIGH);
      executedOffSteps++;
      // Serial.println("Main & Off Step");
    } else {
      // Serial.println("Main Step");
    }
    delayMicroseconds(STEP_DELAY);
    digitalWrite(mainStepPin, LOW);
    digitalWrite(offStepPin, LOW);  // We can always just reset the offStepPin
    delayMicroseconds(STEP_DELAY);
  }
  // After we are done we check if offSteps - executedOffSteps == 0. If it does we are good. If not we need to execute one more off step.
  // offSteps - executedOffSteps should only ever be 0 or 1.
  if (offSteps - executedOffSteps > 0) {
    digitalWrite(offStepPin, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(offStepPin, LOW);
    delayMicroseconds(STEP_DELAY);
    executedOffSteps++;
  }
  // Serial.print("Offs: ");
  // Serial.print(offSteps);
  // Serial.print(", ");
  // Serial.print(executedOffSteps);
  // Serial.print(" (Step Error: ");
  // Serial.print(offSteps - executedOffSteps);
  // Serial.println(")");
  // Serial.println("");
}

// Example implementations
void move_to_initial_position(int x, int y) {
  // Call your existing function to move the actuators to the initial position
  char buffer[256];
  sprintf(buffer, "INFO:X: %d, Y: %d, DX: %d, DY: %d, Current: (%d, %d)", x, y, x - currentX, y - currentY, currentX, currentY);
  Serial.println(buffer);
  executeDelta(x - currentX, y - currentY);
  currentX = x;
  currentY = y;
}

void draw_binary_data(byte *controlPointsHex, unsigned int numControlPoints) {
  // Call your existing "draw" function with the binary data and length
  char buffer[256];
  sprintf(buffer, "INFO:Drawing command sequence of length %d", numControlPoints);
  Serial.println(buffer);
  
  for (int currentControlPoint = 0; currentControlPoint < numControlPoints; currentControlPoint++) {
    unsigned int x_lsb = controlPointsHex[4*currentControlPoint] & 0xFF;
    unsigned int x_msb = (controlPointsHex[4*currentControlPoint + 1] & 0xFF) << 8;
    unsigned int x = x_lsb + x_msb;
    unsigned int y_lsb = controlPointsHex[4*currentControlPoint + 2] & 0xFF;
    unsigned int y_msb = (controlPointsHex[4*currentControlPoint + 3] & 0xFF) << 8;
    unsigned int y = y_lsb + y_msb;

    sprintf(buffer, "INFO:X: %d, Y: %d, DX: %d, DY: %d, Current: (%d, %d)", x, y, x - currentX, y - currentY, currentX, currentY);
    Serial.println(buffer);

    executeDelta(x - currentX, y - currentY);
    currentX = x;
    currentY = y;
  }
}

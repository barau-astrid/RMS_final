// INCLUDE ALL THE NECESSARY LIBRARIES
#include <AccelStepper.h>  // for smooth acceleration and deceleration control of the stepper motors
#include <Servo.h>         // it is a standard library to control servo motors (for us, for the Z axis)
#include <math.h>

// Arduino pins definition for the stepper drivers and servo
#define EN_ALL 8        // enables all 4 stepper drivers at once (turned on when on LOW)
#define SERVO_Z_PIN 9   // Z axis servo signal using Pulse Width Modulation (PWM) to indicate what angle to move to
#define X_STEP 2        // X axis stepper driver
#define X_DIR 5         // X axis direction, i.e. which way to rotate the motor
#define Y_STEP 3        // Y axis stepper driver
#define Y_DIR 6         // Y axis direction, i.e. which way to rotate the motor
#define TOOL_A_STEP 4   // sends pulses to one of the 2 tool motors (screwing and changing tools)
#define TOOL_A_DIR 7    // controls the direction for the tool motor A
#define TOOL_B_STEP 10  // sends pulses to the other tool motor (screwing and changing tools)
#define TOOL_B_DIR 11   // controls the direction for the tool motor B
// Both motor combined form a differential drive system.

const float X_MAX_SPEED = 500.0;     // maximum speed for the X axis - steps/s - safe but usable speed
const float X_ACCEL = 200.0;         // acceleration for the X axis - steps/s² - prevents jerky movements (lower: smoother but slower)
const float Y_MAX_SPEED = 500.0;     // maximum speed for the Y axis - steps/s - safe but usable speed
const float Y_ACCEL = 200.0;         // acceleration for the Y axis - steps/s² - prevents jerky movements
const float TOOL_MAX_SPEED = 400.0;  // maximum speed for the tool motors
const float TOOL_ACCEL = 200.0;      // acceleration for the tool motors

// Number of steps per millimeter for each stepper motor
const float STEPS_PER_MM_X = 1.0;
const float STEPS_PER_MM_Y = 1.0;

// Servo angles for the Z axis for all stages: the higher the angle, the lower the position on the setup
const int Z_UP = 30;          // safe travel height, i.e. the end-effector is high enough not to have anything in its way
const int Z_APPROACH = 60;    // height just above a screw head, i.e. not touching but ready to touch and screw
const int Z_DOWN = 100;       // fully pressed down and engaged with the screw (2 types of M4 screws)
const int Z_SERVO_MIN = 0;    // minimum allowed servo angle - limit set for safety (physical protection)
const int Z_SERVO_MAX = 180;  // maximum allowed servo angle - avoids getting out of the mechanical range

const unsigned long SERVO_SETTLE_MS = 400;  // wait after every servo move so the system physically arrives at destination - ms
const unsigned long MOVE_DELAY_MS = 100;    // short pause after XY moves to avoid jerk (no mechanical vibrations left)

// The robot assumes it starts at the specified positions when powered on.
// It has to be placed manually at the corner
const float HOME_X = 0.0;  // mm
const float HOME_Y = 0.0;  // mm

// 20x20 cm physical build area — any move outside of the area gets rejected for safety
// This constitutes a "soft limit" to prevent crashes into physical boundaries.
const float WS_MIN_X = 0.0;    // X axis minimum boundary - mm
const float WS_MAX_X = 200.0;  // X axis maximum boundary - mm
const float WS_MIN_Y = 0.0;    // Y axis minimum boundary - mm
const float WS_MAX_Y = 200.0;  // Y axis maximum boundary - mm
// The circular part to screw sits in the workspace defined just above
const float BC_CX = 120.0;  // X coordinate of the center of the circular part - mm - shifted to the right for the screws parking slot
const float BC_CY = 100.0;  // Y coordinate of this center - mm
const float BC_R = 65.0;    // radius of the circle for the screws - mm (not the radius of the circular part but the circle with the screws within the circular part)
// Screw parking slot (the screws are stacked vertically on the left side of the workspace)
const float SLOT_X = 22.0;        // X coordinate for all screws parking slots (vertical column) - mm
const float SLOT_Y_START = 28.0;  // Y coordinate for the first screw parking slot - mm
const float SLOT_Y_STEP = 28.0;   // gap between each screw parking slot in the Y axis - mm

// Number of steps to switch end-effectors
const long SELECTOR_POS_BIT1 = 0;           // step position for bit 1, i.e. screw M4 type 1 (no rotation from start position)
const long SELECTOR_POS_BIT2 = 200;         // step position for bit 2, i.e. screw M4 type 2 (200 steps rotation compared to the position for bit 1)
const long SCREW_ROTATION_STEPS_M3 = 1000;  // number of steps to install an M4 type 1 screw
const long SCREW_ROTATION_STEPS_M4 = 1000;  // number of steps to install an M4 type 2 screw

enum ScrewType { M3 = 3,
                 M4 = 4 };  // special data type for the screw - initially size, afterwards type

// Hereunder is created the data structure to group information for each screw:
// identifier, type, position in the circle, in the parking slots, required screw end-effector
struct ScrewData {
  int id;              // identifier (from 0 to 5 as we have 6 screws)
  ScrewType type;      // the type of M4 screw
  float origX, origY;  // position on the circular part
  float targX, targY;  // position in the parking slot on the left side of the workspace
  int bitNumber;       // 1 = former M3 - type 1 M4 bit, 2 = type 2 M4 bit - for the side of the end-effector
};

const int NUM_SCREWS = 6;      // total number of screws
ScrewData screws[NUM_SCREWS];  //array of 6 ScrewData elements, i.e. one for each of our screws

float degToRad(float d) {
  return d * PI / 180.0;
}  // converts degrees to radians for cos and sin operations

// Function which, when the robot starts, compiles all the data on the screws:
// positions (computed, not set, to fit changes) aso
// Even-indexed screws are what used to be M3, i.e. now M4 type 1, odd-indexed are M4 type 2
// Both types alternate along the circle
void buildScrewTable() {
  for (int i = 0; i < NUM_SCREWS; i++) {
    float angleDeg = i * 60.0;                                // there is one screw every 60 degrees (to fit the 6 at equal distance and angle from one another)
    screws[i].id = i;                                         // index of the considered screw
    screws[i].origX = BC_CX + BC_R * cos(degToRad(angleDeg)); // position on the circle for the X axis (X = centerX of the circle + radius × cos(angle))
    screws[i].origY = BC_CY + BC_R * sin(degToRad(angleDeg)); // position on the circle for the Y axis (Y = centerY of the circle + radius × sin(angle))
    screws[i].targX = SLOT_X;                                 // parking slot X coordinate
    screws[i].targY = SLOT_Y_START + i * SLOT_Y_STEP;         // parking slot Y coordinate
    if (i % 2 == 0) {           // check if the screw is even or odd numbered to determine its type
      screws[i].type = M3;      // event number: before, type M3, now type M4 n1
      screws[i].bitNumber = 1;  // bit number associated to the type of screw (1 for M3, 2 for M4)
    } else {                    // same operations but for the other type of screws
      screws[i].type = M4;
      screws[i].bitNumber = 2;
    }
  }
}

// Creation of the acceleration steppers, with the mode, step and direction pins as parameters; DRIVER means that we use the step/directing signal
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR);             // X axis
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP, Y_DIR);             // Y axis
AccelStepper toolMotorA(AccelStepper::DRIVER, TOOL_A_STEP, TOOL_A_DIR); // first tool motor A
AccelStepper toolMotorB(AccelStepper::DRIVER, TOOL_B_STEP, TOOL_B_DIR); // second tool motor B

Servo servoZ;   // servo for the Z axis

// We track the current position as there is no sensor in the robot to give feedback on its own position.
float currentX_mm = HOME_X;                   // mm
float currentY_mm = HOME_Y;                   // mm
int currentZPos = Z_UP;                       // servo angle
int currentBit = 1;                           // current screwdriver (1 at the start)
long currentSelectorPos = SELECTOR_POS_BIT1;  // step position for the type of screw

// Safety function to check whether the robot is within the safe boundaries of the workspace
bool isInsideWorkspace(float x, float y) {
  return x >= WS_MIN_X && x <= WS_MAX_X && y >= WS_MIN_Y && y <= WS_MAX_Y;
}

// Safety function to prevent the servo from going past physical limits, i.e. mechanical range by turning any value into the safe range
int clampServo(int p) {   // clamp = restrict to a value in the valid range
  if (p < Z_SERVO_MIN) return Z_SERVO_MIN;
  if (p > Z_SERVO_MAX) return Z_SERVO_MAX;
  return p;
}

// Function to move along the Z axis to a given angle (clamping and settling time included)
void moveZ(int pos) {
  pos = clampServo(pos);  // change the values into safe ones
  servoZ.write(pos);      // send the command to the servo
  currentZPos = pos;      // update the position along the Z axis
  delay(SERVO_SETTLE_MS); // wait for the servo to move in the real world
}

// Function to move in the XY plane, while remaining in the safe zone of the workspace and having the position along the Z axis high enough
void moveToXY(float x, float y) {
  if (!isInsideWorkspace(x, y)) {                 // check to see if we are aiming for a position in the safe zone
    Serial.print(F("SKIP outside workspace: "));
    Serial.print(x);
    Serial.print(',');
    Serial.println(y);
    return;
  }
  if (currentZPos != Z_UP) moveZ(Z_UP);           // safety measure to move in the XY plane while being high enough only
  long sx = (long)((x - currentX_mm) * STEPS_PER_MM_X); // compute the needed number of steps to move along the X axis
  long sy = (long)((y - currentY_mm) * STEPS_PER_MM_Y); // compute the needed number of steps to move along the Y axis
  stepperX.move(sx);  // gives the command on the number of steps to achieve along the X axis, the sign indicating the direction of the movement
  stepperY.move(sy);  // gives the command on the number of steps to achieve along the Y axis, the sign indicating the direction of the movement
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();   // one step along the X axis
    stepperY.run();   // one step along the Y axis
  }
  currentX_mm = x;      // new x position
  currentY_mm = y;      // new y position
  delay(MOVE_DELAY_MS); // pause to let the vibrations from the movement end
}

// Function to return the robot to its starting position
void goHome() {
  moveZ(Z_UP);
  moveToXY(HOME_X, HOME_Y);
  Serial.println(F("At home"));
}

// Function to deal with the end-effector: when both motors turn in the same direction, change tools; in the same, screw
// motor A = selector + driver,  motorB = selector - driver
void commandTool(long sel, long drv) {  // sel is the movement of the selector (bit switching), drv is the movement of the driver (screwing)
  toolMotorA.move(sel + drv);
  toolMotorB.move(sel - drv);
  while (toolMotorA.distanceToGo() != 0 || toolMotorB.distanceToGo() != 0) {  // to have both motors complete their movements
    toolMotorA.run();
    toolMotorB.run();
  }
  delay(MOVE_DELAY_MS); // wait to let everything finish
}

// Function to change screw drivers' bits when needed
void selectBit(int bit) {
  if (bit < 1 || bit > 2 || bit == currentBit) return;  // return if the number is not in the possible ones or already the right one
  long target = (bit == 1) ? SELECTOR_POS_BIT1 : SELECTOR_POS_BIT2; // if the bit is 1, use SELECTOR_POS_BIT1, else SELECTOR_POS_BIT2
  commandTool(target - currentSelectorPos, 0);                      // move the selector to the target setting
  currentSelectorPos = target;                                      // update of the selector position
  currentBit = bit;                                                 // update of the current bit
  Serial.print(F("Bit selected: "));
  Serial.println(bit);
}

// Function to drive the screw, which includes at the same time going down the Z axis; and unscrew, so also going up the Z axis
void screwWithZ(long driverSteps, int zStart, int zEnd) { // driverSteps is how many steps to rotate (positive=clockwise); zStart and zEnd the servo positions at the beginning and end
  zStart = clampServo(zStart);    // safety check
  zEnd = clampServo(zEnd);        // idem
  toolMotorA.move(driverSteps);   // screwing/unscrewing means motors A and B should rotate in the opposite directions
  toolMotorB.move(-driverSteps);
  long total = abs(driverSteps);  // total number of steps to complete, without regard to the direction
  if (total == 0) {
    servoZ.write(zEnd);
    currentZPos = zEnd;
    return;
  }
  while (toolMotorA.distanceToGo() != 0 || toolMotorB.distanceToGo() != 0) {  // while the motor still has steps to accomplish
    toolMotorA.run();
    toolMotorB.run();
    long done = total - abs(toolMotorA.distanceToGo()); // number of steps already completed
    float t = (float)done / (float)total;               // progress, from 0.0 to 1.0
    servoZ.write(clampServo(zStart + (int)(t * (float)(zEnd - zStart)))); // linear interpolation
  }
  servoZ.write(zEnd);     // ensure the servo is at the target position
  currentZPos = zEnd;     // update the position along the Z axis
  delay(SERVO_SETTLE_MS); // wait until the end of the movement
}

// Function to get the rotation steps for each screw type
long getSteps(ScrewType t) {
  return (t == M3) ? SCREW_ROTATION_STEPS_M3 : SCREW_ROTATION_STEPS_M4;
}

// Function to unscrew at a given position: go there, get down, unscrew, go up
void unscrewAt(float x, float y, int bit, int zDown, long steps) {  // x, y is the screw position, bit the type of end-effector, zDown how low to go, steps the needed number of rotations
  Serial.print(F("Unscrew at x:"));
  Serial.print(x, 1);
  Serial.print(F(" y:"));
  Serial.println(y, 1);
  moveToXY(x, y);         // move in the XY plane, to finish above the localisation of the screw
  selectBit(bit);         // change to the right screw driver
  Serial.println(F("Z to approach"));
  moveZ(Z_APPROACH);      // go down along the Z axis, right above the screw
  Serial.println(F("Z to down engage"));
  moveZ(zDown);           // press down first to fully seat the bit in before spinning
  Serial.println(F("Unscrewing Z rises"));
  screwWithZ(-steps, zDown, Z_APPROACH);  // negative number of steps to unscrew
  Serial.println(F("Z to up"));
  moveZ(Z_UP);            // go up along the Z axis with the screw
}

// Function to screw at a given position: go there, get down, screw, go up
void screwAt(float x, float y, int bit, int zDown, long steps) {  // same parameters as unscrewAt
  Serial.print(F("Screw at x:"));
  Serial.print(x, 1);
  Serial.print(F(" y:"));
  Serial.println(y, 1);
  moveToXY(x, y);         // move in the XY plane, to finish above the localisation of the screw
  selectBit(bit);         // change to the right screw driver
  Serial.println(F("Z to approach"));
  moveZ(Z_APPROACH);      // go down along the Z axis, right above the screw
  Serial.println(F("Screwing Z presses down"));
  screwWithZ(steps, Z_APPROACH, zDown);  // positive number of steps to screw
  Serial.println(F("Z to up"));
  moveZ(Z_UP);            // go up along the Z axis with the screw
}

// Step 1: take all screws back out of the parking and return them to their original holes
void processForwardTransfer() {
  Serial.println(F("RETURN column to circle"));
  for (int i = 0; i < NUM_SCREWS; i++) {
    ScrewData &s = screws[i];
    Serial.print(F("Screw "));
    Serial.print(i);
    Serial.print(F(" M"));
    Serial.print((int)s.type);
    Serial.print(F(" from slot ("));
    Serial.print(s.targX, 1);
    Serial.print(F(","));
    Serial.print(s.targY, 1);
    Serial.print(F(") to ("));
    Serial.print(s.origX, 1);
    Serial.print(F(","));
    Serial.print(s.origY, 1);
    Serial.println(F(")"));

    unscrewAt(s.targX, s.targY, s.bitNumber, Z_DOWN, getSteps(s.type));
    screwAt(s.origX, s.origY, s.bitNumber, Z_DOWN, getSteps(s.type));
  }
  Serial.println(F("Return done"));
}

// Step 2: take all screws out of the circular part and place them in their parking slots
void processReturnTransfer() {
  Serial.println(F("FORWARD circle to column"));
  for (int i = 0; i < NUM_SCREWS; i++) {
    ScrewData &s = screws[i];   // creates a reference to the currently dealt with screw
    Serial.print(F("Screw "));
    Serial.print(i);
    Serial.print(F(" M"));
    Serial.print((int)s.type);
    Serial.print(F(" from ("));
    Serial.print(s.origX, 1);   // source X coordinate
    Serial.print(F(","));
    Serial.print(s.origY, 1);   // source Y coordinate
    Serial.print(F(") to slot ("));
    Serial.print(s.targX, 1);   // goal X coordinate
    Serial.print(F(","));
    Serial.print(s.targY, 1);   // goal Y coordinate
    Serial.println(F(")"));

    unscrewAt(s.origX, s.origY, s.bitNumber, Z_DOWN, getSteps(s.type)); // unscrew the screw in the circle
    screwAt(s.targX, s.targY, s.bitNumber, Z_DOWN, getSteps(s.type));   // screw it in its parking slot
  }
  Serial.println(F("Forward done"));
}

// Main function to run the entire demonstration
void performFullCycle() {
  processForwardTransfer(); // step 1
  delay(1000);              // 1s pause between steps, in the back and forth
  processReturnTransfer();  // step 2
  goHome();                 // let the robot return to its starting position
  Serial.println(F("Full cycle done"));
}

// Function to establish the coordinate system origin - home position (placed manually)
void initializeHomePosition() {
  stepperX.setCurrentPosition(0);   // no movement, just setting of the counter
  stepperY.setCurrentPosition(0);   // idem
  toolMotorA.setCurrentPosition(0); // idem
  toolMotorB.setCurrentPosition(0); // idem
  currentX_mm = HOME_X;                   // set the current X position as the home one
  currentY_mm = HOME_Y;                   // set the current Y position as the home one
  currentBit = 1;                         // set the current bit as 1, as for ancient M3, new M4 type 1 bit
  currentSelectorPos = SELECTOR_POS_BIT1; // set the current selector to match the bit
  servoZ.write(Z_UP);       // move the Z servo up
  currentZPos = Z_UP;       // update the current Z position
  delay(SERVO_SETTLE_MS);   // wait for the end of the movement
  Serial.println(F("Home initialized"));
}

// Function for the Arduino setup, which runs every time the board is either reset or powered on
void setup() {
  Serial.begin(115200);             // usual baudrate - bits/s
  pinMode(EN_ALL, OUTPUT);          // OUTPUT as we send a signal out, we do not receive it
  digitalWrite(EN_ALL, LOW);        // LOW to enable all stepper drivers, hence power motors

  stepperX.setMaxSpeed(X_MAX_SPEED);
  stepperX.setAcceleration(X_ACCEL);
  stepperY.setMaxSpeed(Y_MAX_SPEED);
  stepperY.setAcceleration(Y_ACCEL);
  toolMotorA.setMaxSpeed(TOOL_MAX_SPEED);
  toolMotorA.setAcceleration(TOOL_ACCEL);
  toolMotorB.setMaxSpeed(TOOL_MAX_SPEED);
  toolMotorB.setAcceleration(TOOL_ACCEL);
  servoZ.attach(SERVO_Z_PIN);

  buildScrewTable();        // computes the positions of the 6 screws with the given information (center, radius aso)

  initializeHomePosition();

  // Print the full coordinate table so you can verify positions before anything moves
  // Debugging feature: see all calculated positions before robot starts moving.
  Serial.println(F("Screw coordinate table"));  // safety&debugging: print every positions before the robot starts moving
  for (int i = 0; i < NUM_SCREWS; i++) {
    Serial.print(F("S")); // screw index
    Serial.print(i);
    Serial.print(F(" M"));
    Serial.print((int)screws[i].type);
    Serial.print(F(" bit"));
    Serial.print(screws[i].bitNumber);
    Serial.print(F(" orig("));
    Serial.print(screws[i].origX, 1); // position in the circle along the X axis
    Serial.print(F(","));
    Serial.print(screws[i].origY, 1); // position in the circle along the Y axis
    Serial.print(F(") slot("));
    Serial.print(screws[i].targX, 1); // position in the parking slots along the X axis
    Serial.print(F(","));
    Serial.print(screws[i].targY, 1); // position in the parking slots along the Y axis
    Serial.println(F(")"));
  }
  delay(2000);  // 2s break to check, like a safety delay to give time to reset or depower the robot

  performFullCycle(); // if all is good, the robot is set into motion
}

void loop() {}  // not used here because everything is set up from the start of the robot, and afterwards it just stops
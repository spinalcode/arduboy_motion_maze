//#define GRAYSCALE
//#define USEMOTION

#ifdef GRAYSCALE
#define ABG_IMPLEMENTATION
#include "ArduboyG.h"
ArduboyG a;
#else
#include <Arduboy2.h>
Arduboy2 a;
#endif

#include <Wire.h>

#define TILTMAX 16
#define MAPWIDTH 32
#define MAPHEIGHT 19

const int MPU = 0x68; // MPU6050 I2C address
float x, y = 0;

float pitchOffset = 0;
float rollOffset = 0;

float ballX = 64;
float ballY = 32;
float velocityX = 0;
float velocityY = 0;

int frameNumber = 0;
const float dampingFactor = 0.95; // Damping factor to decay speed

int offsetX, offsetY = 0;
uint8_t tileSize = 8;
uint8_t tilesX, tilesY;

const uint8_t all_levels[] PROGMEM = {
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,3,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,1,1,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,
0,0,1,1,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,
0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

const uint8_t ball[] PROGMEM = {
11, 11,
0x70, 0xfc, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfa, 0xf2, 0xcc, 0x70, 
0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 
};



void setup() {

  a.begin();

#ifdef GRAYSCALE
  a.startGray();
#endif
  
  tilesX = a.width() / tileSize;
  tilesY = a.height() / tileSize;

  power_twi_enable(); // Enable TWI (I2C) power
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Configure the gyroscope
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x18); // set +/- 2000 degrees/sec
  Wire.endTransmission(true);
  
  // Configure the accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x18); // set +/- 16g
  Wire.endTransmission(true);

  calibrateMPU();
}

void loop() {
  if(!a.nextFrame()) return;
  if(++frameNumber == 3) frameNumber = 0;

  if(frameNumber == 0) {
    readMPU();
    y -= pitchOffset;
    if(y> TILTMAX)y= TILTMAX;
    if(y<-TILTMAX)y=-TILTMAX;
    x -= rollOffset;
    if(x> TILTMAX)x= TILTMAX;
    if(x<-TILTMAX)x=-TILTMAX;
  }

  if(frameNumber == 1) {
    // Update velocity based on joystick values
    velocityX += x * 0.0125;
    velocityY += y * 0.0125;

    // Apply damping to velocity
    velocityX *= dampingFactor;
    velocityY *= dampingFactor;

    // Update ball position based on velocity
    ballX += velocityX;
    ballY += velocityY;

    // Check for collision with screen edges and reverse velocity
    if (ballX <= 0 || ballX >= ((MAPWIDTH * tileSize) - 1)) {
      velocityX = -velocityX;
      ballX = constrain(ballX, 0, (MAPWIDTH * tileSize) - 1);
    }
    if (ballY <= 0 || ballY >= ((MAPHEIGHT * tileSize) - 1)) {
      velocityY = -velocityY;
      ballY = constrain(ballY, 0, (MAPHEIGHT * tileSize) - 1);
    }

    // Update offsets to account for the screen's center position
    offsetX = ballX - (a.width() / 2);
    offsetY = ballY - (a.height() / 2);
  }

  // Draw the maze with adjusted offsets
  drawMaze();

  // Draw the ball relative to the screen center
  int ballScreenX = -offsetX + ballX;
  int ballScreenY = -offsetY + ballY;
  Sprites::drawOverwrite(ballScreenX, ballScreenY, ball, 0);

#ifndef GRAYSCALE
a.display();
#endif
}




void drawTile(int x, int y, int color) {
  // Check if tile is within screen bounds
  if(x < -8 || x > 127 || y < -8 || y > 63) return;
  
  // Determine the clipping of the tile
  int tsx = 8, tsy = 8;
  if(x < 0) { tsx += x; x = 0; }
  if(x > 120) tsx = 127 - x;
  if(y < 0) { tsy += y; y = 0; }
  if(y > 56) tsy = 63 - y;
  
  // Draw the tile
#ifdef GRAYSCALE
  a.fillRect(x, y, tsx, tsy, color);
#else
  a.fillRect(x, y, tsx, tsy, color);
#endif
}

void drawMaze() {
#ifndef GRAYSCALE
a.clear();
#endif
  int startX = max(0, (offsetX / tileSize));
  int endX = min(31, (offsetX + 128) / tileSize);
  int startY = max(0, (offsetY / tileSize));
  int endY = min(19, (offsetY + 64) / tileSize);

  for (int y1 = startY; y1 <= endY; y1++) {
    int y2 = y1 * tileSize;
    int ty = y2 - offsetY + y;
    for (int x1 = startX; x1 <= endX; x1++) {
      int x2 = x1 * tileSize;
      int tx = x2 - offsetX + x;
      uint8_t tile = pgm_read_byte(&all_levels[x1 + 32 * y1]);
      if(tile != 0){
        drawTile(tx, ty, 1);
      }
    }
  }

  for (int y1 = startY; y1 <= endY; y1++) {
    int y2 = y1 * tileSize;
    int ty = y2 - offsetY;
    for (int x1 = startX; x1 <= endX; x1++) {
      int x2 = x1 * tileSize;
      int tx = x2 - offsetX;
      uint8_t tile = pgm_read_byte(&all_levels[x1 + 32 * y1]);
      if(tile != 0){
        drawTile(tx, ty, 3);
      }
    }
  }
}


void readMPU() {

#ifndef USEMOTION
  int ballSpeed = TILTMAX;
  x = 0;
  if(a.pressed(RIGHT_BUTTON)){ x = ballSpeed; }
  if(a.pressed(LEFT_BUTTON)){ x = -ballSpeed; }
  y = 0;
  if(a.pressed(UP_BUTTON)){ y = -ballSpeed; }
  if(a.pressed(DOWN_BUTTON)){ y = ballSpeed; }
  return;
#endif
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // request a total of 14 registers

  int16_t AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int16_t AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
/*
  Wire.read(); Wire.read(); // Skip temperature registers
  Wire.read(); Wire.read(); // Skip Gyro
  Wire.read(); Wire.read(); // Skip Gyro
  Wire.read(); Wire.read(); // Skip Gyro
*/  
  // Convert raw values to degrees/second for gyroscope and g for accelerometer
  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;

  // Calculate pitch and roll using accelerometer data
  float pitch = atan2(ay, az) * 180 / PI;
  float roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  y = pitch/4;
  x = roll/4;
}

void calibrateMPU() {

#ifdef USEMOTION

#ifndef GRAYSCALE
a.clear();
#endif

  while (!a.pressed(A_BUTTON)){
    if(a.nextFrame()){
      a.clear();
      a.print(F("Press A to calibrate"));
#ifndef GRAYSCALE
a.display();
#endif
    }
  }

  for (int t = 5; t > 0; t--) {
    int st = millis();
    while(millis() < st+1000){
      if(a.nextFrame()){
        a.clear();
        a.print(t);
#ifndef GRAYSCALE
a.display();
#endif
      }
    }
  }
  readMPU();
  pitchOffset = y;
  rollOffset = x;

#endif

}

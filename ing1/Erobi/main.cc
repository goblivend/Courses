// #include <Servo.h>
#include <math.h>
#include <iostream>

/* Array storing the 12 servos */
// Servo servos[4][3]; // 4 legs of 3 segments each
/* Array of pins connected to the servo */
const int servo_pin[4][3] = {
  { 4, 2, 3 }, // Leg 1 - Front right (R2) - {H, F, T}
  { 7, 5, 6 }, // Leg 2 - Back right (R1) - {H, F, T}
  { 16, 14, 15 }, // Leg 3 - Front left (L1) - {H, F, T}
  { 19, 17, 18 } // Leg 4 - Back left (L2) - {H, F, T}
};

const double PI = 3.14159265358979323846;
double a = 53;
double b = 79.5;
double c = 30.5;

double rad2deg(double rad) {
  return rad * (180/PI);
}

double getGamma(double x, double y, double z) {
  // std::cout << "    Computing gamma for " << x << " " << y << " " << z << std::endl;
  return rad2deg(atan2(y, x));
}

double getAlpha(double x, double y, double z) {
  // std::cout << "    Computing alpha for " << x << " " << y << " " << z << std::endl;
  double v = sqrt(x*x + y*y) - c;
  double d = sqrt(v*v + z*z);

  double alp1 = atan2(z, v);
  double alp2 = acos((a*a + d*d - b*b)/(2*a*d));

  double alp = alp1 + alp2;

  return rad2deg(alp);
}

double getBeta(double x, double y, double z) {
  // std::cout << "    Computing beta for " << x << " " << y << " " << z << std::endl;
  double v = sqrt(x*x + y*y) - c;
  double d = sqrt(v*v + z*z);

  return rad2deg(acos((a*a + b*b - d*d)/(2*a*b)));
}

// Hanche
double getActualGamma(double angle, int leg) {
  // std::cout << "    Computing actual gamma for " << angle << " " << leg << std::endl;
  switch (leg) {
  case 0: // R2
    return 90 + angle;
  case 1: // R1
    return 90 - angle;
  case 2: // L1
    return 90 - angle;
  case 3: // L2
    return 90 + angle;
  default:
    return 180;
  }
}

// Fémur
double getActualAlpha(double angle, int leg) {
  switch (leg) {
  case 0: // R2
    return 90 - angle;
  case 1: // R1
    return 90 + angle;
  case 2: // L1
    return 90 + angle;
  case 3: // L2
    return 90 - angle;
  default:
    return 180;
  }
}

// Tibia
double getActualBeta(double angle, int leg) {
  if (leg == 1 || leg == 2) // R2 L1
    return 140 - angle;
  else                      // R1 L2
    return angle;
}


void toPLS() {
  std::cout << "PLS" << std::endl;
  int tibias[4] = {160, 20, 20, 160};
  for (int i = 0; i < 4; i++) {
    // servos[i][2].write(tibias[i]);
    std::cout << "Servo " << i << " 2 : " << tibias[i] << std::endl;
  }

//   delay(200);

  for (int i = 0; i < 4; i++) {
    // servos[i][1].write(90);
    std::cout << "Servo " << i << " 1 : " << 90 << std::endl;
    // servos[i][0].write(90);
    std::cout << "Servo " << i << " 0 : " << 90 << std::endl;
  }

  exit(0); // End program if PLS called
}

const double physical_coordinates[4][3] = {
  {109, 60, 42}, // R2
  {110, 43, 41}, // R1
  {108, 71, 70}, // L1
  {107, 61, 67}, // L2
};

double calib[4][3] = {
  {0, 0, 0},  // R2
  {0, 0, 0},  // R1
  {0, 0, 0},  // L1
  {0, 0, 0}   // L2
};

double fixAngle(double angle, int leg, int servo) {
  return angle + calib[leg][servo];
}

double getAngle(int x, int y, int z, int leg, int servo) {
  // std::cout << "  Computing Servo " << leg << " " << servo << " : " << x << " " << y << " " << z << std::endl;
  double angle = 0;
  switch (servo) {
  case 0:
    angle = getActualGamma(getGamma(x, y, z), leg);
    break;
  case 1:
    angle = getActualAlpha(getAlpha(x, y, z), leg);
    break;
  case 2:
    angle = getActualBeta(getBeta(x, y, z), leg);
    break;
  }
  // std::cout << "  Angle : " << angle << std::endl;

  return fixAngle(angle, leg, servo);
}

void get_calib() {
  int x = 100;
  int y = 70;
  int z = 15;
  int delta_z = 27;
  for (int leg = 0; leg < 4; leg++) {
    for (int servo = 0; servo < 3; servo++) {
      double expected = getAngle(x, y, z, leg, servo);
      double actual = getAngle(physical_coordinates[leg][0],
                               physical_coordinates[leg][1],
                               physical_coordinates[leg][2] - delta_z,
                               leg, servo);
      calib[leg][servo] = actual - expected;
      // std::cout << "Calibration for Servo " << leg << " " << servo << " : " << calib[leg][servo] << std::endl;
    }
  }
}


bool isInRange(double angle, int leg, int servo) {
  int limits[4][3][2] = {
    {{60, 175}, {5 , 120}, {40, 160}}, // R2
    {{5 , 120}, {60, 175}, {20, 140}}, // R1
    {{5 , 120}, {60, 175}, {20, 140}}, // L1
    {{60, 175}, {5 , 120}, {40, 160}}, // L2
  };
  // std::cout << "[RANGE] Checking if Servo " << leg << " " << servo << " : " << angle << " is  in [" << limits[leg][servo][0] << ", " << limits[leg][servo][1] << "]" << std::endl;
  return limits[leg][servo][0] <= angle && angle <= limits[leg][servo][1];
}

void move(double angle, int leg, int servo) {
  std::cout << "Trying Servo " << leg << " " << servo << " : " << angle << std::endl;
  if (!isInRange(angle, leg, servo))
    toPLS();

//   servos[leg][servo].write(angle);
//   delay(1000);
}

void moveTo(int x, int y, int z, int leg) {
  // std::cout << "[MOVE] Moving to " << x << " " << y << " " << z << " " << leg << std::endl;
  for (int servo = 0; servo < 3; servo++) {
    move(getAngle(x, y, z, leg, servo), leg, servo);
  }
}

void setup() {
  get_calib();
  for (int i = 0; i < 4; i++) {
    moveTo(100, 70, 15, i);
  }

}

int main() {
  setup();
}

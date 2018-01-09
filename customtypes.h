#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

struct Pose {
  public:
    Pose(float x_in, float y_in, float phi_in) : x(x_in), y(y_in), phi(phi_in) {};
    float x;
    float y;
    float phi;
};

struct Point {
  public:
    Point(float x_in, float y_in) : x(x_in), y(y_in) {};
    float x;
    float y;
};

namespace CONSTANTS {
  const float RAD2DEG = 57.2957795131;
  const float X_MAX_METER = 3.;
  const float Y_MAX_METER = 3.;
  const float CAR_WIDTH_METER = 0.05;
  const float CAR_LENGTH_METER = 0.1;
}


#endif // CUSTOMTYPES_H

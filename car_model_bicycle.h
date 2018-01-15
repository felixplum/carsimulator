#ifndef CAR_MODEL_BICYCLE_H
#define CAR_MODEL_BICYCLE_H

#include "car.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "customtypes.h"

class CarModelBicycle : public Car
{
public:
    CarModelBicycle(const Map& map_global);
    void EvaluateModel(const std::vector<float>& state_vec,
                const std::vector<float>& control_input_vec,
                std::vector<float>* evaluation_vec) const;
    void GetControl(std::vector<float>* u_out);
    void GetWaypointsPixel(std::vector<Point>* wp_out) const;

    std::vector<Point> waypoint_vec_local_pixel_;
    std::vector<Point> waypoint_vec_local_meter_;
private:
    bool UpdateWaypoints();
};

#endif // CAR_MODEL_BICYCLE_H

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
    void GetControl(const std::vector<Point>& waypoints,
                    std::vector<float>* u_out) const;
};

#endif // CAR_MODEL_BICYCLE_H

#ifndef CAR_MODEL_BICYCLE_H
#define CAR_MODEL_BICYCLE_H

#include "car.h"
#include <iostream>
#include <vector>
#include <algorithm>

class CarModelBicycle : public Car
{
public:
    CarModelBicycle();
    void EvaluateModel(const std::vector<float>& state_vec,
                const std::vector<float>& control_input_vec,
                std::vector<float>* evaluation_vec) const;
};

#endif // CAR_MODEL_BICYCLE_H

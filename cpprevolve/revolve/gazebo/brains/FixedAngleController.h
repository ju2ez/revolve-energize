//
// Created by Matteo De Carlo on 11/09/2019.
//

#ifndef GAZEBO_REVOLVE_FIXEDANGLECONTROLLER_H
#define GAZEBO_REVOLVE_FIXEDANGLECONTROLLER_H


#include <revolve/brains/controller/actuators/Actuator.h>
#include <revolve/brains/controller/actuators/sensors/Sensor.h>
#include <revolve/brains/controller/FixedAngleController.h>
#include <revolve/gazebo/motors/ActuatorWrapper.h>
#include "Brain.h"

namespace revolve
{
namespace gazebo
{

class FixedAngleController: public revolve::FixedAngleController
{
public:
    explicit FixedAngleController(double angle)
        : revolve::FixedAngleController(angle)
    {}

public: bool angle_to_target_below_threshold(double threshold);

};

    bool FixedAngleController::angle_to_target_below_threshold(double threshold) {
        return false;
    }

}
}


#endif //GAZEBO_REVOLVE_FIXEDANGLECONTROLLER_H

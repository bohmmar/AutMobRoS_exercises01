#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Saturation.hpp>
//#include <eeros/control/Constant.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> q1;       // control exercise servo motor
    Gain<> g;
    PeripheralOutput<> servo;   // control exercise servo motor
    PeripheralInput<> E2;       // for encoder exercise 1

    PeripheralOutput<> M1;      // Ex3 Ang. Vel
    Saturation<> QMax;         // Ex3 Ang. Vel
    Gain<> Scale;       // calculate to rad
    Gain<> i;           // reduction ratio
    Gain<> km;          // motor constant
    Gain<> R;           // resistance
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP
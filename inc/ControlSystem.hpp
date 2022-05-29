#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Sum.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E2;       // for encoder exercise 1
    PeripheralInput<> E1;

    Saturation<> QMax;         // Ex3 Ang. Vel
    Saturation<> qmax;
    D<> ed;                     // Differentiator
    D<> qd1;                     // Differentiator

    Sum<> e;
    Sum<> qdd;
    Sum<> U1;
    
    Gain<> iInv;           // inverse reduction ratio
    Gain<> kmInv;          // inverse motor constant
    Gain<> R;           // resistance
    Gain<> kp;          // proportional gain
    Gain<> kd;          // d Gain
    Gain<> M;
    Gain<> km;
    Gain<> i;

    PeripheralOutput<> M1;      // Ex3 Ang. Vel
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP
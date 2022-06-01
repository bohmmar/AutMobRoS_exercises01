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
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Mux.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1;       // for encoder exercise 1
    PeripheralInput<> E2;

    Saturation<eeros::math::Vector2> QMax;         // Ex3 Ang. Vel
    Saturation<eeros::math::Vector2> qmax;

    D<eeros::math::Vector2> ed;                     // Differentiator
    D<eeros::math::Vector2> qd1;                     // Differentiator

    Mux<2> Mu1;
    Mux<2> Mu2;
    DeMux<2> DeMu1;

    Sum<2,eeros::math::Vector2> e;
    Sum<2,eeros::math::Vector2> qdd;
    Sum<2,eeros::math::Vector2> U1;
    
    Gain<eeros::math::Vector2> iInv;           // inverse reduction ratio
    Gain<eeros::math::Vector2> kmInv;          // inverse motor constant
    Gain<eeros::math::Vector2> R;           // resistance
    Gain<eeros::math::Vector2> kp;          // proportional gain
    Gain<eeros::math::Vector2> kd;          // d Gain
    Gain<eeros::math::Vector2> M;
    Gain<eeros::math::Vector2> km;
    Gain<eeros::math::Vector2> i;

    PeripheralOutput<> M1;      // Ex3 Ang. Vel
    PeripheralOutput<> M2;      // Ex3 Ang. Vel
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP
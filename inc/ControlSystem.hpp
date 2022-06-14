#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include "customBlocks/Controller.hpp"
#include "customBlocks/InvMotMod.hpp"
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Constant.hpp>
#include "customBlocks/InvKin.hpp"
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;

    D<eeros::math::Vector2> Ed;                     // Differentiator
    Mux<2> E;
    FwKinOdom fwKinOdom;
    Constant<> RvRx, omegaR;
    InvKin invKin;
    D<> E1d;
    Gain<> g;
    Controller<> controller;
    InvMotMod<> invMotMod;
    PeripheralOutput<> M1;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP
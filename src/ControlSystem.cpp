#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), servo("servo1"),         // exercise with servo motor
      E2("enc2"), i(104.0/3441.0), Scale(0.03/2.0/3.141), QMax(0.1), km(1/8.44e-3), R(8), M1("motor1"),      // exercise encoder
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");       // exercise with servo motor
    g.setName("g");         // exercise with servo motor
    servo.setName("servo");     // exercise with servo motor
    E2.setName("E2");           // exercise encoder
    R.setName("R");             // Ex3 Torque
    i.setName("i");             // Ex3 Torque
    Scale.setName("Scale");     // Ex3 Torque
    km.setName("km");           // Ex3 Torque
    M1.setName("M1");           // Ex3 Torque

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");     // exercise with servo motor
    g.getOut().getSignal().setName("alpha");        // exercise with servo motor
    E2.getOut().getSignal().setName("q2[rad]");     // exercise encoder
    Scale.getOut().getSignal().setName("Q1[Nm]");      // Ex3 Torque
    i.getOut().getSignal().setName("T1[Nm]");       // Ex3 Torque
    km.getOut().getSignal().setName("I1[A]");          // Ex3 Torque
    QMax.getOut().getSignal().setName("Q1[Nm]");     // Ex3 Torque
    R.getOut().getSignal().setName("U1[V]");            // Ex3 Torque



    // Connect signals
    g.getIn().connect(q1.getOut());     // exercise with servo motor
    servo.getIn().connect(g.getOut());      // exercise with servo motor
    Scale.getIn().connect(E2.getOut());     // Ex3 Torque
    QMax.getIn().connect(Scale.getOut());  // Ex3 Torque
    i.getIn().connect(QMax.getOut());      // Ex3 Torque
    km.getIn().connect(i.getOut());         // Ex3 Torque
    R.getIn().connect(km.getOut());         // Ex3 Torque
    M1.getIn().connect(R.getOut());        // Ex3 Torque
    

    // Add blocks to timedomain
    timedomain.addBlock(q1);        // exercise with servo motor
    timedomain.addBlock(g);         // exercise with servo motor
    timedomain.addBlock(servo);     // exercise with servo motor
    timedomain.addBlock(E2);        // exercise encoder
    timedomain.addBlock(QMax);     // Ex3 Torque
    timedomain.addBlock(Scale);     // Ex3 Torque
    timedomain.addBlock(i);         // Ex3 Torque
    timedomain.addBlock(km);        // Ex3 Torque
    timedomain.addBlock(R);        // Ex3 Torque

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}
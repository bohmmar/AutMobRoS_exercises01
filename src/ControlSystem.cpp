#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), servo("servo1"),         // exercise with servo motor
      E2("enc2"), i(3441.0/104.0), toRad(21.2/2.0/3.141), qdMax(21.2), km(8.44e-3), M1("motor1"),      // exercise encoder
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");       // exercise with servo motor
    g.setName("g");         // exercise with servo motor
    servo.setName("servo");     // exercise with servo motor
    E2.setName("E2");           // exercise encoder
    i.setName("i");             // Ex2 Ang. Vel
    toRad.setName("toRad");     // Ex2 Ang. Vel
    km.setName("km");           // Ex2 Ang. Vel
    M1.setName("M1");           // Ex2 Ang. Vel

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");     // exercise with servo motor
    g.getOut().getSignal().setName("alpha");        // exercise with servo motor
    E2.getOut().getSignal().setName("q2[rad]");     // exercise encoder
    toRad.getOut().getSignal().setName("qd1[rad/s]");      // Ex2 Ang. Vel
    i.getOut().getSignal().setName("omega1");       // Ex2 Ang. Vel
    km.getOut().getSignal().setName("U1[V]");          // Ex2 Ang. Vel
    qdMax.getOut().getSignal().setName("qd1s[rad/s]");     // Ex2 Ang. Vel



    // Connect signals
    g.getIn().connect(q1.getOut());     // exercise with servo motor
    servo.getIn().connect(g.getOut());      // exercise with servo motor
    toRad.getIn().connect(E2.getOut());     // Ex2 Ang. Vel
    qdMax.getIn().connect(toRad.getOut());  // Ex2 Ang. Vel
    i.getIn().connect(toRad.getOut());      // Ex2 Ang. Vel
    km.getIn().connect(i.getOut());         // Ex2 Ang. Vel
    M1.getIn().connect(km.getOut());        // Ex2 Ang. Vel
    

    // Add blocks to timedomain
    timedomain.addBlock(q1);        // exercise with servo motor
    timedomain.addBlock(g);         // exercise with servo motor
    timedomain.addBlock(servo);     // exercise with servo motor
    timedomain.addBlock(E2);        // exercise encoder
    timedomain.addBlock(qdMax);     // Ex2 Ang. Vel
    timedomain.addBlock(toRad);     // Ex2 Ang. Vel
    timedomain.addBlock(i);         // Ex2 Ang. Vel
    timedomain.addBlock(km);        // Ex2 Ang. Vel

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}
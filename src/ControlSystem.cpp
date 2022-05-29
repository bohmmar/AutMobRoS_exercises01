#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : 
    E1("enc1"),
    E2("enc2"),
    kp(1.0/dt/4.4/0.7/dt/4.4/0.7),
    kd(2.0*0.7/dt/4.4/0.7),
    iInv(104.0/3441.0),
    M(3441/104*3441/104*6.8e-8),
    QMax(0.1),
    kmInv(1/8.44e-3),
    R(8),
    M1("motor1"),      
    timedomain("Main time domain", dt, true)
{
    // Name all blocks

    E2.setName("E2");          
    E1.setName("E1");           
    kp.setName("kp");
    kd.setName("kd");
    M.setName("M");
    R.setName("R");            
    iInv.setName("i");            
    kmInv.setName("km");           
    M1.setName("M1");          
    qdd.setName("qdd");
    e.setName("e");
    ed.setName("ed");
    M1.setName("M1");


    // Name all signals

    E1.getOut().getSignal().setName("q1[rad]");
    E2.getOut().getSignal().setName("q2[rad]"); 
    e.getOut().getSignal().setName("e[rad]");
    kp.getOut().getSignal().setName("qdd_cp[rad/s^2]");
    ed.getOut().getSignal().setName("ed [rad/s");
    kd.getOut().getSignal().setName("qdd_cd[rad/s^2]");
    qdd.getOut().getSignal().setName("qdd[rad/s^2]");
    M.getOut().getSignal().setName("Q1[Nm]");
    QMax.getOut().getSignal().setName("Q1[Nm]");     
    iInv.getOut().getSignal().setName("T1[Nm]");       
    kmInv.getOut().getSignal().setName("I1[A]");                
    R.getOut().getSignal().setName("U1[V]");            



    // Connect signals

    e.getIn(0).connect(E1.getOut());
    e.getIn(1).connect(E1.getOut());
    e.negateInput(1);
    kp.getIn().connect(e.getOut());
    ed.getIn().connect(e.getOut());
    kd.getIn().connect(ed.getOut());
    qdd.getIn(0).connect(kp.getOut());
    qdd.getIn(1).connect(kd.getOut());
    M.getIn().connect(qdd.getOut());
    QMax.getIn().connect(M.getOut());
    iInv.getIn().connect(QMax.getOut());      
    kmInv.getIn().connect(iInv.getOut());         
    R.getIn().connect(kmInv.getOut());         
    M1.getIn().connect(R.getOut());        
    

    // Add blocks to timedomain

    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(e);
    timedomain.addBlock(kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(kd);
    timedomain.addBlock(qdd);
    timedomain.addBlock(M);       
    timedomain.addBlock(QMax);     
    timedomain.addBlock(iInv);         
    timedomain.addBlock(kmInv);       
    timedomain.addBlock(R);  
    timedomain.addBlock(M1);      

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}
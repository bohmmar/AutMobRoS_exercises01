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
    qmax(15.8),
    kmInv(1/8.44e-3),
    R(8),
    i(3441.0/104.0),
    km(8.44e-3),
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
    i.setName("i");
    iInv.setName("iInv");            
    kmInv.setName("kmInv");
    km.setName("km");          
    M1.setName("M1");          
    qdd.setName("qdd");
    e.setName("e");
    ed.setName("ed");
    qd1.setName("qd1");
    U1.setName("U1");
    M1.setName("M1");
    QMax.setName("Qmax");
    qmax.setName("qmax");


    // Name all signals
    E1.getOut().getSignal().setName("q1[rad]");
    E2.getOut().getSignal().setName("q2[rad]"); 
    e.getOut().getSignal().setName("e[rad]");
    kp.getOut().getSignal().setName("qdd_cp[rad/s^2]");
    ed.getOut().getSignal().setName("ed [rad/s");
    qd1.getOut().getSignal().setName("qd1 [rad/s");
    kd.getOut().getSignal().setName("qdd_cd[rad/s^2]");
    qdd.getOut().getSignal().setName("qdd[rad/s^2]");
    M.getOut().getSignal().setName("Q1[Nm]");
    QMax.getOut().getSignal().setName("Q1[Nm]");
    qmax.getOut().getSignal().setName("qd1[rad/s]");       
    iInv.getOut().getSignal().setName("T1[Nm]");
    i.getOut().getSignal().setName("w1 [rad/s]");       
    kmInv.getOut().getSignal().setName("I1[A]");
    km.getOut().getSignal().setName("U1_c [V]");                
    R.getOut().getSignal().setName("U1[V]");            



    // Connect signals
    e.getIn(0).connect(E2.getOut());
    e.getIn(1).connect(E1.getOut());
    e.negateInput(1);
    kp.getIn().connect(e.getOut());
    ed.getIn().connect(e.getOut());
    qd1.getIn().connect(E1.getOut());
    kd.getIn().connect(ed.getOut());
    qdd.getIn(0).connect(kp.getOut());
    qdd.getIn(1).connect(kd.getOut());
    M.getIn().connect(qdd.getOut());
    QMax.getIn().connect(M.getOut());
    qmax.getIn().connect(qd1.getOut());
    iInv.getIn().connect(QMax.getOut());
    i.getIn().connect(qmax.getOut());       
    kmInv.getIn().connect(iInv.getOut()); 
    km.getIn().connect(i.getOut());        
    R.getIn().connect(kmInv.getOut());
    U1.getIn(0).connect(R.getOut());
    U1.getIn(1).connect(km.getOut());         
    M1.getIn().connect(R.getOut());        
    

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(e);
    timedomain.addBlock(kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(qd1);
    timedomain.addBlock(kd);
    timedomain.addBlock(qdd);
    timedomain.addBlock(M);       
    timedomain.addBlock(QMax);
    timedomain.addBlock(qmax);     
    timedomain.addBlock(iInv);
    timedomain.addBlock(i);         
    timedomain.addBlock(kmInv);
    timedomain.addBlock(km);       
    timedomain.addBlock(R); 
    timedomain.addBlock(U1); 
    timedomain.addBlock(M1);      

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}
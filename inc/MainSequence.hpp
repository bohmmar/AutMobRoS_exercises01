#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MyRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include "customSteps/moveServoTo.hpp"

class MainSequence : public eeros::sequencer::Sequence
{
public:
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 MyRobotSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

          sleep("Sleep", this),
          moveServoTo("moveServoTo", this, cs)
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            moveServoTo(-0.5);
            sleep(1.0);
            moveServoTo(0.5);
            sleep(1.0);
            
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    MyRobotSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
    MoveServoTo moveServoTo;
};

#endif // MAINSEQUENCE_HPP_
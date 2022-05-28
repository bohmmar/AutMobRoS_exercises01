#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MyRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include "customSteps/moveServoTo.hpp"
#include "customSequences/orientationException.hpp"

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
          moveServoTo("moveServoTo", this, cs),

          checkOrientation(0.1, cs),
          orientationException("Orientation exception", this, cs, checkOrientation)
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
    CheckOrientation checkOrientation;
    OrientationException orientationException;
};

#endif // MAINSEQUENCE_HPP_
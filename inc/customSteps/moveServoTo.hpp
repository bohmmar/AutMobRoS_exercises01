#ifndef MOVESERVOTO_HPP_
#define MOVESERVOTO_HPP_

#include <eeros/sequencer/Step.hpp>
#include "Controlsystem.hpp"

class MoveServoTo : public eeros::sequencer::Step
{
public:
    MoveServoTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : cs(cs), eeros::sequencer::Step(name, caller)
    {
        log.info() << "Step created: " << name;
    }

    int operator() (double c)
    {
        this->c =c;
        return start();
    }

    int action()
    {
        // do something
        log.info() << "Moving to: " << c << "rad.";
        cs.c.setValue(c);
        return 0;
    }

private:
    // Define variables, conditions, monitors, exception sequences, ...
    ControlSystem &cs;
    double c;
};

#endif // MOVESERVOTO_HPP_

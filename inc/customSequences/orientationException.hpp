#ifndef ORIENTATIONEXCEPTION_HPP_
#define ORIENTATIONEXCEPTION_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Condition.hpp>


class CheckOrientation : public eeros::sequencer::Condition
{
public:
    CheckOrientation(double angle, ControlSystem &cs) 
        : angle(angle), cs(cs) {}

    bool validate() { return abs(cs.g.getOut().getSignal().getValue()) > angle; }

private:
    // Define steps, sub-sequences, variables, conditions, monitors, exception sequences, ...
    ControlSystem &cs;
    double angle;
};


class OrientationException : public eeros::sequencer::Sequence
{
public:
    OrientationException(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs, CheckOrientation checkOrientation)
        : cs(cs), checkOrientation(checkOrientation),
        eeros::sequencer::Sequence(name, caller, true)
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        // do something
        log.warn() << "Orientation around x is outsdide of the allowed range!";
        return 0;
    }

    bool ckeckExitCondition()
    {
        return !checkOrientation.validate();
    }

private:
    // Define steps, sub-sequences, variables, conditions, monitors, exception sequences, ...
    ControlSystem &cs;
    CheckOrientation checkOrientation;
};

#endif // ORIENTATIONEXCEPTION_HPP_

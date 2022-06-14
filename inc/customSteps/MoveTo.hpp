#ifndef MOVETO_HPP_
#define MOVETO_HPP_

#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include "ControlSystem.hpp"

class MoveTo : public eeros::sequencer::Step
{
public:
    /**
     * @brief Construct a new Move To object
     *
     * @param name name of the step
     * @param caller caller
     * @param cs control system
     */
    MoveTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : eeros::sequencer::Step(name, caller),
        cs(cs)
    {
        log.info() << "Step created: " << name;
    }

    /**
     * @brief operator override
     *
     * @param xT x coordinate of target position
     * @param yT y coordinate of target position
     * @param phiT target orientation
     * @return int start the step
     */

    int operator()(double xT, double yT, double phiT)
    {
        this->xT = xT;
        this->yT = yT;
        this->phiT = phiT;
        return start();
    }


    /**
     * @brief action method
     * Set new target position and output it on the console.
     *
     * @return int 0
     */
    int action()
    {
        cs.pp.setTarget(xT, yT, phiT);
        log.info() << "Target pose set to x: " << xT << ", y: " << yT << ", phi: " << phiT;
        log.info() << "Start moving.";
        return 0;
    }

    bool checkExitCondition()
    {
        if (cs.pp.getStatus())
            log.info() << "Target pose reached.";
        return cs.pp.getStatus();
    }
private:
    // Define variables, conditions, monitors, exception sequences, ...
    double xT, yT, phiT;
    ControlSystem &cs;
};

#endif // MOVETO_HPP_

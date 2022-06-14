#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
// Include header files for the subblocks

using namespace eeros::control;

template <typename T = double>
class Controller : public Block   // Set the number of inputs and outputs
{
public:
    /**
     * @brief Construct a new Controller object
     *
     * @param om0 natural frequency
     * @param D lehr's damping ratio
     * @param M mass matrix
     * @param eLimit integrator limit
     */
    Controller(double om0, double D, double M, T eLimit)
        : qd(this),
          Kp(2.0 * D * om0),
          Ki(om0 * om0),
          M(M)
    {
        init(eLimit);
    } 

    /**
     * @brief Construct a new Controller object
     *
     * @param fTask task frequency
     * @param D lehr's damping ratio
     * @param s safety factor
     * @param M mass matrix
     * @param eLimit integrator limit
     */

    Controller(double fTask, double D, double s, double M, T eLimit)
        : qd(this),
          Kp(fTask / s ),
          Ki(fTask / 2.0 / s / D * fTask / 2.0 / s / D),
          M(M)
    {
        init(eLimit);
    }

    /**
     * @brief Get the In object
     *
     * @param index index
     * @return Input<T>& index 0: qd_d, index 1: qd
     */

    virtual Input<T> &getIn(uint8_t index)
    {
        if (index == 0)
        {
            return ed.getIn(0);
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    /**
     * @brief Get the Out object
     *
     * @param index index
     * @return Output<T>& index 0: Q, index 1: qd
     */

    virtual Output<T> &getOut(uint8_t index)
    {
        if (index == 0)
        {
            return M.getOut();
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    // Implement getter functions for the subsystem inputs

    /**
     * @brief run method
     *
     */

    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks
        e.run();
        Kp.run();
        ed.run();
        Ki.run();
        qdd_c.run();
        M.run();
    }

    /**
     * @brief enable integrator
     * 
     */
    void enable()
    {
        e.enable();
    }

    /**
     * @brief disable integrator
     * 
     */
    void disable()
    {
        e.disable();
    }

    /**
     * @brief sets the position error limit
     * 
     * @param eLimit position error limit
     */
    void setELimit(T eLimit)
    {
        e.setLimit(eLimit, -eLimit);
    }

protected:
    // Define intermediate variables and subblocks
    InputSub<T> qd;
    Sum<2, T> ed, qdd_c;
    Gain<T> Kp, Ki, M;
    I<T> e;

private:
    /**
     * @brief init method
     *
     * @param eLimit integrator limit
     */
    void init(T eLimit)
    {
        // Name all blocks
        ed.setName("ed");
        Kp.setName("Kp");
        e.setName("e");
        Ki.setName("Ki");
        qdd_c.setName("qdd_c");
        M.setName("M");

        // Name all signals
        ed.getOut().getSignal().setName("ed [rad/s]"); 
        Kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
        e.getOut().getSignal().setName("e [rad]");
        Ki.getOut().getSignal().setName("qdd_ci [rad/s^2]");
        qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
        M.getOut().getSignal().setName("Q [Nm]");
        
        // Connect signals
        ed.getIn(1).connect(qd);
        ed.negateInput(1);
        Kp.getIn().connect(ed.getOut());
        e.getIn().connect(ed.getOut());
        Ki.getIn().connect(e.getOut());   
        qdd_c.getIn(0).connect(Kp.getOut());
        qdd_c.getIn(1).connect(Ki.getOut());
        M.getIn().connect(qdd_c.getOut());

        // Additional configuration
        T eInit = 0.0;
        e.setInitCondition(eInit);
        e.setLimit(eLimit, -eLimit);
    }
};

#endif //CONTROLLER_HPP_

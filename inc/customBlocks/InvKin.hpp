#ifndef INVKIN_HPP_
#define INVKIN_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>
// Include header files for the subblocks

using namespace eeros::control;

/**
 * @brief Inverse kinematics class for a differential drive robot
 * 
 */


class InvKin : public Block   // Set the number of inputs and outputs
{
public:
    /**
     * @brief Construct a new Inv Kin object
     * 
     * @param B distance between the two wheels
     */
    InvKin(double B) 
        : B(B),
          WJR([&, B]()
              {
              eeros::math::Matrix<2,2> M = {1.0, 1.0, -B/2.0, B/2.0};
              eeros::math::Vector2 v = {WJR.getIn(0).getSignal().getValue(), WJR.getIn(1).getSignal().getValue()};
              WJR.getOut().getSignal().setValue(M*v);
              WJR.getOut().getSignal().setTimestamp(WJR.getIn(0).getSignal().getTimestamp()); })
    {
        // Name all blocks
        WJR.setName("InvKin->WJR");

        // Name all signals
        WJR.getOut().getSignal().setName("vW [m/s]");
    }

    /**
     * @brief Input getter function
     * 
     * @return Input<>& Input for the robot velocity in x direction
     */
    Input<> &getInRvRx() { return WJR.getIn(0); }

    /**
     * @brief Input getter function
     * 
     * @return Input<>& Input for the angular robot velocity
     */
    Input<> &getInOmegaR() { return WJR.getIn(1); }

    /**
     * @brief Ouput getter function
     * 
     * @return Output<eeros::math::Vector2>& Output for the wheel velocities
     */
    Output<eeros::math::Vector2> &getOut() { return WJR.getOut(); }

    /**
     * @brief run method
     * 
     */
    virtual void run()
    {
        WJR.run();
    }

protected:
    double B;
    Blockio<2, 1, double, eeros::math::Vector2> WJR;
};

#endif //INVKIN_HPP_

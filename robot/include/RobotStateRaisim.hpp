#pragma once

#include "RobotState.hpp"
#include <raisim/World.hpp>

/**
 * @brief Implementation of RobotState for Raisim environment.
 */
class RobotStateRaisim : public RobotState {
public:
    RobotStateRaisim(raisim::World* world, raisim::ArticulatedSystem* robot);
    ~RobotStateRaisim();

    // Override pure virtual functions
    void updateState() override;
   
    
    
private:
    raisim::World* world_;               // Pointer to Raisim world
    raisim::ArticulatedSystem* robot_;   // Pointer to Raisim ArticulatedSystem
};
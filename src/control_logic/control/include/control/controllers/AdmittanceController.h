#pragma once
#include "BaseController.h"
#include "ControlTypes.h"  // for data structures like JointState, etc.

class AdmittanceController : public BaseController
{
public:
    AdmittanceController();
    ~AdmittanceController() override = default;

    // Initialize gains, load config, etc.
    bool init(const ControllerConfig& config) override;

    // Update loop called each cycle
    void update(const JointState& current_state,
                const ExternalForces& forces,
                JointCommand& command_out) override;

private:
    // Gains, RCM params if directly enforced here, etc.
};

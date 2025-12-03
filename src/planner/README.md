# Planner Module

Non–real-time planning daemon for the Merai stack.

## Responsibilities

- Attach to `/ParameterServerShm`, `/RTDataShm`, `/LoggerShm`.
- Build `robot_lib::RobotModel` from `merai::ParameterServer`.
- Poll `plannerRequestBuffer` for `PENDING` jobs.
- For `JOINT_TRAJECTORY` jobs:
  - run `planner::JointTrajectoryPlanner`,
  - write `JointTrajectoryPlan` to `jointTrajectoryPlanBuffer`,
  - write `PlannerResult` to `plannerResultBuffer`.
- Log via shared logger.

No networking, no GUI, no RT loop here.

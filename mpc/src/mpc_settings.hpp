#ifndef KNOT_POINTS 
#define KNOT_POINTS 32 
#endif

#ifndef TIMESTEP //timestep between knot points (seconds)
#define TIMESTEP 0.125
#endif

// ---------------------- trajectory_publisher.cpp ----------------------

#ifndef TRAJ_PUBLISH_PERIOD_MS // Wait time for publishing trajectories (trajectory_publisher.cpp)
#define TRAJ_PUBLISH_PERIOD_MS 8000 //currently 8 seconds so we can see robot_driver traj logic
#endif


// ---------------------- robot_driver_jpos.cpp ----------------------

#ifndef ROBOT_CONTROL_PERIOD_MS // Wait time for sending controls and reading state (robot_driver_jpos.cpp)
#define ROBOT_CONTROL_PERIOD_MS 200
#endif
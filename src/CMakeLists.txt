# The platform-independent part
add_subdirectory(common)
add_subdirectory(robot_plan)
add_subdirectory(supervisor)
add_subdirectory(simulated_robot)

# Depends on the actual platform
add_subdirectory(kuka_lcm)
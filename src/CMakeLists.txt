# The platform-independent part
add_subdirectory(common)
add_subdirectory(simulated_robot)

# Depends on the actual platform
add_subdirectory(kuka_lcm)
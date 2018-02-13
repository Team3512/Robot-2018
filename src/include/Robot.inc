#pragma once

#include <iostream>

#include "Constants.hpp"

template <int N>
auto GenerateTrajectory(std::array<Waypoint, N>& waypoints) {
    TrajectoryCandidate candidate;
    pathfinder_prepare(&waypoints[0], N, FIT_HERMITE_CUBIC,
    		PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0,
                       &candidate);

    int length = candidate.length;
    auto trajectory = std::make_unique<Segment[]>(length);

    pathfinder_generate(&candidate, trajectory.get());

    auto leftTrajectory = std::make_unique<Segment[]>(length);
    auto rightTrajectory = std::make_unique<Segment[]>(length);

    pathfinder_modify_tank(trajectory.get(), length, leftTrajectory.get(),
                           rightTrajectory.get(), kWheelbaseWidth);

    for (int i = 0; i < length; i++) {
        Segment s = trajectory[i];
        std::cout << "Time Step: " << s.dt << "\n";
        std::cout << "Coords: (" << s.x << ", " << s.y << ")\n";
        std::cout << "Position (distance): " << s.position << "\n";
        std::cout << "Velocity: " << s.velocity << "\n";
        std::cout << "Acceleration: " << s.acceleration << "\n";
        std::cout << "Jerk (acceleration per second): " << s.jerk << "\n";
        std::cout << "Heading (radians): " << s.heading << "\n";
    }
    return std::make_tuple(std::move(trajectory), std::move(leftTrajectory), std::move(rightTrajectory));
}
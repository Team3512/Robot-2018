// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/SumNode.h"

#include <cmath>

using namespace frc;

/**
 * Appends input and associated sign to array.
 *
 * @param input the INode object to add to the array for round robin
 * @param positive if true, adds input; otherwise, subtracts input
 */
SumNode::SumNode(INode& input, bool positive) {
    m_inputs.emplace_back(input, positive);
}

double SumNode::GetOutput() {
    double sum = 0.0;

    for (auto& input : m_inputs) {
        if (input.second) {
            sum += input.first.GetOutput();
        } else {
            sum -= input.first.GetOutput();
        }
    }

    std::scoped_lock lock(m_mutex);

    m_lastResult = m_currentResult;
    m_currentResult = sum;

    if (m_continuous && m_inputRange != 0) {
        sum = std::fmod(sum, m_inputRange);
        if (std::abs(sum) > m_inputRange / 2.0) {
            if (sum > 0.0) {
                return sum - m_inputRange;
            } else {
                return sum + m_inputRange;
            }
        }
    }

    return sum;
}

/**
 * Set the sum node to consider the input to be continuous.
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the reference (e.g., gyroscope angle).
 *
 * @param continuous true turns on continuous; false turns off continuous
 */
void SumNode::SetContinuous(bool continuous) {
    std::scoped_lock sync(m_mutex);
    m_continuous = continuous;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the input
 */
void SumNode::SetInputRange(double minimumInput, double maximumInput) {
    std::scoped_lock sync(m_mutex);
    m_inputRange = maximumInput - minimumInput;
}

/**
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param tolerance absolute error which is tolerable
 * @param deltaTolerance change in absolute error which is tolerable
 */
void SumNode::SetTolerance(double tolerance, double deltaTolerance) {
    std::scoped_lock lock(m_mutex);

    m_tolerance = tolerance;
    m_deltaTolerance = deltaTolerance;
}

/**
 * Return true if the error and change in error is within the range determined
 * by SetTolerance().
 */
bool SumNode::InTolerance() const {
    std::scoped_lock lock(m_mutex);
    return std::abs(m_currentResult) < m_tolerance &&
           std::abs(m_currentResult - m_lastResult) < m_deltaTolerance;
}

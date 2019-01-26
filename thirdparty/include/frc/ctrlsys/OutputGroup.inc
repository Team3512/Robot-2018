// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc {

/**
 * Appends output to array.
 *
 * @param output the Output object to add to the array for round robin
 * @param outputs the other Output objects
 */
template <class... Outputs>
OutputGroup::OutputGroup(Output& output, Outputs&&... outputs)
    : OutputGroup(outputs...) {
    m_outputs.emplace_back(output);
}

}  // namespace frc

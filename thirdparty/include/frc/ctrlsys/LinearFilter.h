// Copyright (c) 2015-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/ArrayRef.h>
#include <wpi/circular_buffer.h>

#include <vector>

#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/NodeBase.h"

namespace frc {

/**
 * SinglePoleIIR tag type.
 */
class SinglePoleIIR {};

/**
 * HighPass tag type.
 */
class HighPass {};

/**
 * MovingAverage tag type.
 */
class MovingAverage {};

constexpr SinglePoleIIR kSinglePoleIIR{};
constexpr HighPass kHighPass{};
constexpr MovingAverage kMovingAverage{};

/**
 * This class implements a linear, digital filter. All types of FIR and IIR
 * filters are supported. Static factory methods are provided to create commonly
 * used types of filters.
 *
 * Filters are of the form:<br>
 *  y[n] = (b0 * x[n] + b1 * x[n-1] + … + bP * x[n-P]) -
 *         (a0 * y[n-1] + a2 * y[n-2] + … + aQ * y[n-Q])
 *
 * Where:<br>
 *  y[n] is the output at time "n"<br>
 *  x[n] is the input at time "n"<br>
 *  y[n-1] is the output from the LAST time step ("n-1")<br>
 *  x[n-1] is the input from the LAST time step ("n-1")<br>
 *  b0 … bP are the "feedforward" (FIR) gains<br>
 *  a0 … aQ are the "feedback" (IIR) gains<br>
 * IMPORTANT! Note the "-" sign in front of the feedback term! This is a common
 *            convention in signal processing.
 *
 * What can linear filters do? Basically, they can filter, or diminish, the
 * effects of undesirable input frequencies. High frequencies, or rapid changes,
 * can be indicative of sensor noise or be otherwise undesirable. A "low pass"
 * filter smooths out the signal, reducing the impact of these high frequency
 * components.  Likewise, a "high pass" filter gets rid of slow-moving signal
 * components, letting you detect large changes more easily.
 *
 * Example FRC applications of filters:
 *  - Getting rid of noise from an analog sensor input (note: the roboRIO's FPGA
 *    can do this faster in hardware)
 *  - Smoothing out joystick input to prevent the wheels from slipping or the
 *    robot from tipping
 *  - Smoothing motor commands so that unnecessary strain isn't put on
 *    electrical or mechanical components
 *  - If you use clever gains, you can make a PID controller out of this class!
 *
 * For more on filters, I highly recommend the following articles:<br>
 *  http://en.wikipedia.org/wiki/Linear_filter<br>
 *  http://en.wikipedia.org/wiki/Iir_filter<br>
 *  http://en.wikipedia.org/wiki/Fir_filter<br>
 *
 * Note 1: GetOutput() should be called by the user on a known, regular period.
 * You can set up a Notifier to do this (look at the WPILib PIDController
 * class), or do it "inline" with code in a periodic function.
 *
 * Note 2: For ALL filters, gains are necessarily a function of frequency. If
 * you make a filter that works well for you at, say, 100Hz, you will most
 * definitely need to adjust the gains if you then want to run it at 200Hz!
 * Combining this with Note 1 - the impetus is on YOU as a developer to make
 * sure GetOutput() gets called at the desired, constant frequency!
 */
class LinearFilter : public NodeBase {
public:
    LinearFilter(SinglePoleIIR, INode& input, double timeConstant,
                 double period);
    LinearFilter(HighPass, INode& input, double timeConstant, double period);
    LinearFilter(MovingAverage, INode& input, int taps);
    LinearFilter(INode& input, wpi::ArrayRef<double> ffGains,
                 wpi::ArrayRef<double> fbGains);

    double GetOutput() override;

    void Reset(void);

private:
    wpi::circular_buffer<double> m_inputs;
    wpi::circular_buffer<double> m_outputs;
    std::vector<double> m_inputGains;
    std::vector<double> m_outputGains;
};

}  // namespace frc

#pragma once

#include <frc/controllers/StateSpaceControllerCoeffs.h>
#include <frc/controllers/StateSpaceLoop.h>
#include <frc/controllers/StateSpaceObserverCoeffs.h>
#include <frc/controllers/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<1, 1, 1> MakeShooterPlantCoeffs();
frc::StateSpaceControllerCoeffs<1, 1, 1> MakeShooterControllerCoeffs();
frc::StateSpaceObserverCoeffs<1, 1, 1> MakeShooterObserverCoeffs();
frc::StateSpaceLoop<1, 1, 1> MakeShooterLoop();

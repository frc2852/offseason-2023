// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotManager {

    private static Alliance allianceColor;
    private static boolean isSimulation;

    // Perform initial startup checks and store necessary static information
    public static void performStartupChecks() {
        allianceColor = DriverStation.getAlliance();
        if (allianceColor == Alliance.Invalid) {
            DriverStation.reportError("Unable to retrieve the alliance from FMS", false);
        }

        isSimulation = RobotBase.isSimulation();
    }

    // Public getter for the stored alliance color
    public static Alliance getAlliance() {
        return allianceColor;
    }

    public static boolean isSimulation() {
        return isSimulation;
    }
}

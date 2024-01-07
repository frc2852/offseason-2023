// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveDistanceCommand;
import frc.robot.commands.Intake.IntakeOutCommandTimed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class MidScoreDriveBack extends SequentialCommandGroup  {
  public MidScoreDriveBack(DriveSubsystem driveTrainSubsystem, PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
    // new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.HIGH_SCORE), pivotSubsystem);
    addCommands(new IntakeOutCommandTimed(intakeSubsystem, 1));
    addCommands(new DriveDistanceCommand(driveTrainSubsystem, 3.5, true));
  }
}

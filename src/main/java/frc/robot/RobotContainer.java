// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.VisionTracking.CycleGridLeft;
import frc.robot.commands.VisionTracking.CycleGridRight;
import frc.robot.subsystems.NodeSelectionSubsystem;

public class RobotContainer {

  CommandPS4Controller driverController = new CommandPS4Controller(0);

  private final NodeSelectionSubsystem mVisionTrackingSubsystem = new NodeSelectionSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverController.L1().onTrue(new CycleGridLeft(mVisionTrackingSubsystem));
    driverController.R1().onTrue(new CycleGridRight(mVisionTrackingSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

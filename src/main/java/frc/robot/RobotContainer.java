// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.VisionTracking.CycleGridLeft;
import frc.robot.commands.VisionTracking.CycleGridRight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NodeSelectionSubsystem;
import frc.robot.subsystems.VisionTrackingSubsystem;

public class RobotContainer {

	CommandPS4Controller driverController = new CommandPS4Controller(0);

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	private final NodeSelectionSubsystem mNodeSelectionSubsystem = new NodeSelectionSubsystem();
	private final VisionTrackingSubsystem mVisionTrackingSubsystem = new VisionTrackingSubsystem();
	private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem(); 

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		driverController.L1().onTrue(new CycleGridLeft(mNodeSelectionSubsystem));
		driverController.R1().onTrue(new CycleGridRight(mNodeSelectionSubsystem));

		driverController.triangle().whileTrue(new IntakeOutCommand(mIntakeSubsystem));
		driverController.cross().whileTrue(new IntakeCommand(mIntakeSubsystem));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}

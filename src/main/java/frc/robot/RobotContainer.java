// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.Auto.MidScoreDriveBack;
import frc.robot.commands.Auto.MidScoreDriveBackBridge;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Intake.IntakeOutSlowCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.commands.VisionTracking.CycleGridLeft;
// import frc.robot.commands.VisionTracking.CycleGridRight;
// import frc.robot.subsystems.NodeSelectionSubsystem;
// import frc.robot.subsystems.VisionTrackingSubsystem;

public class RobotContainer {

	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(pivotSubsystem);

	// private final NodeSelectionSubsystem nodeSelectionSubsystem = new NodeSelectionSubsystem();
	// private final VisionTrackingSubsystem visionTrackingSubsystem = new VisionTrackingSubsystem();

	CommandPS4Controller driverController = new CommandPS4Controller(Constants.OI.DRIVER_CONTROLLER_PORT);

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

	private final SendableChooser<Command> autoSelection = new SendableChooser<>();
	
	private final  MidScoreDriveBack midScoreDriveBack = new MidScoreDriveBack(driveSubsystem, pivotSubsystem, intakeSubsystem);
	private final  MidScoreDriveBackBridge midScoreDriveBackBridge = new MidScoreDriveBackBridge(driveSubsystem, pivotSubsystem, intakeSubsystem);

	public RobotContainer() {
		configureBindings();
		configureAutoSelection();
	}

	private void configureBindings() {

		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		driveSubsystem.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
                    () -> driveSubsystem.drive(
                            -MathUtil.applyDeadband(driverController.getLeftY(), Constants.OI.DRIVE_DEAD_BAND),
                            -MathUtil.applyDeadband(driverController.getLeftX(), Constants.OI.DRIVE_DEAD_BAND),
                            -MathUtil.applyDeadband(driverController.getRightX(), Constants.OI.DRIVE_DEAD_BAND),
                            true, true),
                    driveSubsystem));

		// driverController.L1().onTrue(new CycleGridLeft(nodeSelectionSubsystem));
		// driverController.R1().onTrue(new CycleGridRight(nodeSelectionSubsystem));

		driverController.L2().whileTrue(new IntakeOutCommand(intakeSubsystem));
		driverController.L1().whileTrue(new IntakeOutSlowCommand(intakeSubsystem));

		driverController.R2().whileTrue(new IntakeCommand(intakeSubsystem));

		// driverController.R1().whileTrue(new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.DRIVE), pivotSubsystem));
		// driverController.square().whileTrue(new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.PICK_UP), pivotSubsystem));
		driverController.cross().whileTrue(new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.PICK_UP), pivotSubsystem));
		driverController.circle().whileTrue(new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.DRIVE), pivotSubsystem));
		driverController.triangle().whileTrue(new RunCommand(() -> pivotSubsystem.setAngle(Constants.Pivot.HIGH_SCORE), pivotSubsystem));
	}

	private void configureAutoSelection(){
		autoSelection.setDefaultOption("Score + Drive", midScoreDriveBack);
		autoSelection.addOption("Score + Bridge", midScoreDriveBackBridge);
		SmartDashboard.putData("Auto Mode", autoSelection);
	  }

	public Command getAutonomousCommand() {
		return autoSelection.getSelected();
	}
}

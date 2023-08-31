// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionTracking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NodeSelectionSubsystem;

/** Add your docs here. */
public class CycleGridRight extends CommandBase {

	private final NodeSelectionSubsystem visionTrackingSubsystem;

	public CycleGridRight(NodeSelectionSubsystem visionTrackingSubsystem) {
		this.visionTrackingSubsystem = visionTrackingSubsystem;
		addRequirements(visionTrackingSubsystem);
	}

	@Override
	public void execute() {
		visionTrackingSubsystem.cycleRight();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}

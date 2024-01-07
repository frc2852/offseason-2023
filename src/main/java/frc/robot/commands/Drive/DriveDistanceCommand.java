// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
  private final DriveSubsystem mDriveSubsystem;
  private final double mRunTime;
  private final boolean mForward;
  private final Timer mTimer = new Timer();

  public DriveDistanceCommand(DriveSubsystem driveSubsystem, double time, boolean forward) {
    this.mDriveSubsystem = driveSubsystem;
    mRunTime = time;
    mForward = forward;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.reset();
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mForward) {
      mDriveSubsystem.drive(0.3, 0, 0, true, true);
    } else {
      mDriveSubsystem.drive(-0.3, 0, 0, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mRunTime);
  }
}

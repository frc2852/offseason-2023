// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

	private final CANSparkMax pivotLeader;
	private final AbsoluteEncoder pivotEncoder;

	private final CANSparkMax pivotFollower;
	private final SparkMaxPIDController pivotPIDController;
	
	private double position = 0;
	public PivotSubsystem() {
		pivotLeader = new CANSparkMax(Constants.PIVOT_LEADER, MotorType.kBrushless);
		pivotFollower = new CANSparkMax(Constants.PIVOT_FOLLOWER, MotorType.kBrushless);
		pivotEncoder = pivotLeader.getAbsoluteEncoder(Type.kDutyCycle);
		pivotPIDController = pivotLeader.getPIDController();

		configureMotors();
		configurePID();
		burnFlash();

		SmartDashboard.putNumber("Position", position);
	}

	private void configureMotors() {
		pivotLeader.restoreFactoryDefaults();
		pivotLeader.setIdleMode(IdleMode.kBrake);
		pivotLeader.setInverted(false);
		// pivotLeader.setSmartCurrentLimit(20);

		pivotFollower.restoreFactoryDefaults();
		pivotFollower.follow(pivotLeader);
		pivotFollower.setIdleMode(IdleMode.kBrake);
		pivotFollower.setInverted(true);
		// pivotFollower.setSmartCurrentLimit(20);
	}

	private void configurePID() {
		pivotEncoder.setInverted(false);
		pivotEncoder.setPositionConversionFactor(360);

		pivotPIDController.setPositionPIDWrappingEnabled(false);
		pivotPIDController.setP(Constants.PIVOT_P);
		pivotPIDController.setI(Constants.PIVOT_I);
		pivotPIDController.setD(Constants.PIVOT_D);
		pivotPIDController.setIZone(Constants.PIVOT_IZONE);
		pivotPIDController.setFF(Constants.PIVOT_FF);
		pivotPIDController.setOutputRange(Constants.PIVOT_MIN_OUTPUT, Constants.PIVOT_MAX_OUTPUT);
		pivotPIDController.setFeedbackDevice(pivotEncoder);
	}

	private void burnFlash() {
		pivotLeader.burnFlash();
		pivotFollower.burnFlash();
	}

	public void setAngle(double angle) {
		pivotPIDController.setReference(angle, ControlType.kPosition);
	}
	
	
	@Override
	public void periodic() {
		SmartDashboard.putNumber("ActualPosition", pivotEncoder.getPosition());
		SmartDashboard.putNumber("PositionConversion", pivotEncoder.getPositionConversionFactor());
		position = SmartDashboard.getNumber("Position", 0);
		setAngle(position);
	}
}

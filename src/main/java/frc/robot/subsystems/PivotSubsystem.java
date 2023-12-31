// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggingManager;
import frc.robot.util.LoggingManager.MessageType;

public class PivotSubsystem extends SubsystemBase {
	private final CANSparkMax pivotLeader;
	private final CANSparkMax pivotFollower;
	
	private final SparkMaxPIDController pivotPIDController;
	private AbsoluteEncoder pivotEncoder;

	private final boolean DEBUG = false;

	public PivotSubsystem() {
		pivotLeader = new CANSparkMax(Constants.CanbusId.PIVOT_LEADER, MotorType.kBrushless);
		pivotFollower = new CANSparkMax(Constants.CanbusId.PIVOT_FOLLOWER, MotorType.kBrushless);
		pivotEncoder = pivotLeader.getAbsoluteEncoder(Type.kDutyCycle);
		if (pivotEncoder == null) {
			LoggingManager.log("Pivot encoder was null", MessageType.ERROR);
			try {
				Thread.sleep(1000);
				pivotEncoder = pivotLeader.getAbsoluteEncoder(Type.kDutyCycle);
			} catch (Exception e) {

			}
		}

		pivotPIDController = pivotLeader.getPIDController();
		// forwardLimit = pivotLeader.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		// reverseLimit = pivotLeader.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		// forwardLimit.enableLimitSwitch(true);
		// reverseLimit.enableLimitSwitch(true);

		configureMotors();
		configurePID();
		burnFlash();

		if (DEBUG) {
			initPIDTune();
		}

		setAngle(Constants.Pivot.DRIVE);
	}

	private void configureMotors() {
		pivotLeader.restoreFactoryDefaults();
		pivotLeader.setIdleMode(IdleMode.kBrake);
		pivotLeader.setInverted(false);

		pivotFollower.restoreFactoryDefaults();
		pivotFollower.follow(pivotLeader);
		pivotFollower.setIdleMode(IdleMode.kBrake);
		pivotFollower.setInverted(true);
	}

	private void configurePID() {
		pivotEncoder.setInverted(false);
		pivotEncoder.setPositionConversionFactor(360);
		
		// double currentOffset = pivotEncoder.getZeroOffset();
		pivotEncoder.setZeroOffset(268.5409904);
		// SmartDashboard.putNumber("Offset", currentOffset);

		pivotPIDController.setP(Constants.Pivot.P);
		pivotPIDController.setI(Constants.Pivot.I);
		pivotPIDController.setD(Constants.Pivot.D);
		pivotPIDController.setIZone(Constants.Pivot.IZONE);
		pivotPIDController.setFF(Constants.Pivot.FF);
		pivotPIDController.setOutputRange(Constants.Pivot.MIN_OUTPUT, Constants.Pivot.MAX_OUTPUT);
		pivotPIDController.setPositionPIDWrappingEnabled(false);
		pivotPIDController.setFeedbackDevice(pivotEncoder);
	}

	private void burnFlash() {
		pivotLeader.burnFlash();
		pivotFollower.burnFlash();
	}

	public void setAngle(double angle) {
		if (angle > 130) {
			angle = 120;
		} else if (angle < 10) {
			angle = 10;
		}

		pivotPIDController.setReference(angle, ControlType.kPosition);
	}

	@Override
	public void periodic() {
		if (DEBUG) {
			tunePID();
		}
	}

	private void initPIDTune() {
		// SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
		// SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimit.isLimitSwitchEnabled());

		// SmartDashboard.putNumber("P Value", Constants.Pivot.P);
		// SmartDashboard.putNumber("I Value", Constants.Pivot.I);
		// SmartDashboard.putNumber("D Value", Constants.Pivot.D);
		// SmartDashboard.putNumber("IZone", Constants.Pivot.IZONE);
		// SmartDashboard.putNumber("FF Value", Constants.Pivot.FF);
		SmartDashboard.putNumber("Min Output", Constants.Pivot.MIN_OUTPUT);
		SmartDashboard.putNumber("Max Output", Constants.Pivot.MAX_OUTPUT);
	}

	private void tunePID() {
		// Fetch PID values from SmartDashboard
		// double pValue = SmartDashboard.getNumber("P Value", Constants.Pivot.P);
		// double iValue = SmartDashboard.getNumber("I Value", Constants.Pivot.I);
		// double dValue = SmartDashboard.getNumber("D Value", Constants.Pivot.D);
		// double izoneValue = SmartDashboard.getNumber("IZone", Constants.Pivot.IZONE);
		// double ffValue = SmartDashboard.getNumber("FF Value", Constants.Pivot.FF);
		double minOutput = SmartDashboard.getNumber("Min Output", Constants.Pivot.MIN_OUTPUT);
		double maxOutput = SmartDashboard.getNumber("Max Output", Constants.Pivot.MAX_OUTPUT);

		// Update PID controller with the fetched values
		// pivotPIDController.setP(pValue);
		// pivotPIDController.setI(iValue);
		// pivotPIDController.setD(dValue);
		// pivotPIDController.setIZone(izoneValue);
		// pivotPIDController.setFF(ffValue);
		pivotPIDController.setOutputRange(minOutput, maxOutput);

		// SmartDashboard.putBoolean("Forward Limit Switch", forwardLimit.isPressed());
		// SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimit.isPressed());
		SmartDashboard.putNumber("ActualPosition", pivotEncoder.getPosition());
		SmartDashboard.putNumber("PositionConversion", pivotEncoder.getPositionConversionFactor());
		// double position = SmartDashboard.getNumber("Position", 0);
		// setAngle(position);
	}
}

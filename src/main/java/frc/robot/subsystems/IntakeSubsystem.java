// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggingManager;
import frc.robot.util.LoggingManager.MessageType;

public class IntakeSubsystem extends SubsystemBase {

	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax topMotor;
	private DigitalInput limitSwitch;

	private double leftSpeed = 0.5;
	private double rightSpeed = 0.5;
	private double topSpeed = 0.5;

	// Variables to store the previous inversion state
	private boolean prevInvertLeft = false;
	private boolean prevInvertRight = false;
	private boolean prevInvertTop = false;

	public IntakeSubsystem() {
		leftMotor = new CANSparkMax(Constants.INTAKE_LEFT, MotorType.kBrushless);
		rightMotor = new CANSparkMax(Constants.INTAKE_RIGHT, MotorType.kBrushless);
		topMotor = new CANSparkMax(Constants.INTAKE_TOP, MotorType.kBrushless);
		limitSwitch = new DigitalInput(0);

		SmartDashboard.putBoolean("Invert Left Motor", false);
		SmartDashboard.putBoolean("Invert Right Motor", false);
		SmartDashboard.putBoolean("Invert Top Motor", false);

		SmartDashboard.putNumber("Left Motor Speed", leftSpeed);
		SmartDashboard.putNumber("Right Motor Speed", rightSpeed);
		SmartDashboard.putNumber("Top Motor Speed", topSpeed);
	}

	public void runIntakeIn() {
		LoggingManager.log(String.format(
				"Running intake: leftSpeed=%.2f (Inverted: %b), rightSpeed=%.2f (Inverted: %b), topSpeed=%.2f (Inverted: %b)",
				leftSpeed,
				leftMotor.getInverted(),
				rightSpeed,
				rightMotor.getInverted(),
				topSpeed,
				topMotor.getInverted()
			), 
			MessageType.DEBUG
		);

		if (!getLimitButton()) {
			leftMotor.set(leftSpeed);
			rightMotor.set(rightSpeed);
			topMotor.set(topSpeed);
		} else {
			stopIntake();
		}
	}

	public void runIntakeOut() {
		LoggingManager.log(String.format(
				"Running intake out: leftSpeed=%.2f (Inverted: %b), rightSpeed=%.2f (Inverted: %b), topSpeed=%.2f (Inverted: %b)",
				leftSpeed, 
				leftMotor.getInverted(), 
				rightSpeed, 
				rightMotor.getInverted(), 
				topSpeed,
				topMotor.getInverted()
			), 
			MessageType.DEBUG
		);

		leftMotor.set(-leftSpeed);
		rightMotor.set(-rightSpeed);
		topMotor.set(-topSpeed);
	}

	public void stopIntake() {
		LoggingManager.log("Intake stopped", MessageType.DEBUG);

		leftMotor.set(0);
		rightMotor.set(0);
		topMotor.set(0);
	}

	public boolean getLimitButton() {
		return limitSwitch.get();
	}

	@Override
	public void periodic() {
		boolean invertLeft = SmartDashboard.getBoolean("Invert Left Motor", false);
		boolean invertRight = SmartDashboard.getBoolean("Invert Right Motor", false);
		boolean invertTop = SmartDashboard.getBoolean("Invert Top Motor", false);

		leftSpeed = SmartDashboard.getNumber("Left Motor Speed", 0.5);
		rightSpeed = SmartDashboard.getNumber("Right Motor Speed", 0.5);
		topSpeed = SmartDashboard.getNumber("Top Motor Speed", 0.5);

		// Only update if the value has changed
		if (invertLeft != prevInvertLeft) {
			leftMotor.setInverted(invertLeft);
			prevInvertLeft = invertLeft;
		}
		if (invertRight != prevInvertRight) {
			rightMotor.setInverted(invertRight);
			prevInvertRight = invertRight;
		}
		if (invertTop != prevInvertTop) {
			topMotor.setInverted(invertTop);
			prevInvertTop = invertTop;
		}
	}
}

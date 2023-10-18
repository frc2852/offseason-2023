// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanbusId;
import frc.robot.util.LoggingManager;
import frc.robot.util.LoggingManager.MessageType;

public class IntakeSubsystem extends SubsystemBase {

	private final PivotSubsystem pivotSubsystem;

	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax topMotor;
	private DigitalInput intakeLimitSwitch;

	private double wheelsIntakeMaxSpeed = 0.35;
	private double rollerIntakeMaxSpeed = 0.35;

	private double wheelsShootMaxSpeed = 1;
	private double rollerShooteMaxSpeed = 1;

	public IntakeSubsystem(PivotSubsystem pivotSubsystem) {
		this.pivotSubsystem = pivotSubsystem;

		leftMotor = new CANSparkMax(CanbusId.INTAKE_LEFT, MotorType.kBrushless);
		leftMotor.setInverted(true);
		leftMotor.setIdleMode(IdleMode.kCoast);

		rightMotor = new CANSparkMax(CanbusId.INTAKE_RIGHT, MotorType.kBrushless);
		rightMotor.setInverted(false);
		rightMotor.setIdleMode(IdleMode.kCoast);

		topMotor = new CANSparkMax(CanbusId.INTAKE_TOP, MotorType.kBrushless);
		topMotor.setInverted(true);
		topMotor.setIdleMode(IdleMode.kCoast);

		leftMotor.burnFlash();
		rightMotor.burnFlash();
		topMotor.burnFlash();

		intakeLimitSwitch = new DigitalInput(0);
	}

	public void runIntakeIn() {
		LoggingManager.log(String.format(
				"Running intake: WheelSpeed=%.2f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.2f (Inverted: %b)",
				wheelsIntakeMaxSpeed,
				leftMotor.getInverted(),
				rightMotor.getInverted(),
				rollerIntakeMaxSpeed,
				topMotor.getInverted()),
				MessageType.DEBUG);

		if (isCubeIntakeComplete()) {
			stopIntake();
			pivotSubsystem.setAngle(Constants.Pivot.DRIVE);
		} else {
			leftMotor.set(wheelsIntakeMaxSpeed);
			rightMotor.set(wheelsIntakeMaxSpeed);
			topMotor.set(rollerIntakeMaxSpeed);
		}
	}

	public void runIntakeOut() {
		LoggingManager.log(String.format(
				"Running intake out: WheelSpeed=%.2f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.2f (Inverted: %b)",
				wheelsIntakeMaxSpeed,
				leftMotor.getInverted(),
				rightMotor.getInverted(),
				rollerIntakeMaxSpeed,
				topMotor.getInverted()),
				MessageType.DEBUG);

		leftMotor.set(-wheelsShootMaxSpeed);
		rightMotor.set(-wheelsShootMaxSpeed);
		topMotor.set(-rollerShooteMaxSpeed);
	}

	public void stopIntake() {
		LoggingManager.log("Intake stopped", MessageType.DEBUG);
		leftMotor.set(0);
		rightMotor.set(0);
		topMotor.set(0);
	}

	public boolean isCubeIntakeComplete() {
		return !intakeLimitSwitch.get();
	}

	@Override
	public void periodic() {

	}
}

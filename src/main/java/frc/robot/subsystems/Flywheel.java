// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.ShooterConstants.SHOOTER_SPEEDS;

public class Flywheel extends SubsystemBase {
	private ThunderBird topMotor;
	private ThunderBird bottomMotor;

	private VelocityVoltage topPID = new VelocityVoltage(0);
	private VelocityVoltage bottomPID = new VelocityVoltage(0);

	private SHOOTER_SPEEDS mode = SHOOTER_SPEEDS.MODERATE;

	private double targetSpeed = 0;

	public Flywheel() {
		topMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD, false, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);
		bottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD, false, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);

		topMotor.configPIDF(0, FlywheelConstants.TOP_0_MOTOR_KP, FlywheelConstants.TOP_0_MOTOR_KI, FlywheelConstants.TOP_0_MOTOR_KD);
		bottomMotor.configPIDF(0, FlywheelConstants.BOTTOM_0_MOTOR_KP, FlywheelConstants.BOTTOM_0_MOTOR_KI, FlywheelConstants.BOTTOM_0_MOTOR_KD);

		topMotor.applyConfig();
		bottomMotor.applyConfig();
	}

	@Override
	public void periodic() {
		LightningShuffleboard.setDouble("Flywheel", "top motor speed", topMotor.getVelocity().getValueAsDouble());
		LightningShuffleboard.setDouble("Flywheel", "bottom motor speed", bottomMotor.getVelocity().getValueAsDouble());
		LightningShuffleboard.setDouble("Flywheel", "target speed", targetSpeed);

		setTopMotorSpeed(targetSpeed);
		setBottomMotorSpeed(targetSpeed);
	}

	public void setTargetSpeed(double speed){
		targetSpeed = speed;
	}

	public void setTopMotorSpeed(double speed){
		topMotor.setControl(topPID.withVelocity(speed).withEnableFOC(false).withSlot(0));
	}

	public void setBottomMotorSpeed(double speed){
		bottomMotor.setControl(bottomPID.withVelocity(speed).withEnableFOC(false).withSlot(0));
	}

	public double getTopMotorSpeed(){
		return topMotor.getVelocity().getValueAsDouble();
	}

	public double getBottomMotorSpeed(){
		return bottomMotor.getVelocity().getValueAsDouble();
	}

	public SHOOTER_SPEEDS getCurrentMode(){
		return mode;
	}

	public double getTargetSpeed(){
		return targetSpeed;
	}

	public void cycleShooterSpeed(){
		switch (mode) {
			case SLOW:
				mode = SHOOTER_SPEEDS.MODERATE;
				break;
			
			case MODERATE:
				mode = SHOOTER_SPEEDS.FAST;
				break;

			case FAST:
				mode = SHOOTER_SPEEDS.SLOW;
				break;

			default:
				break;
		}
	}
}

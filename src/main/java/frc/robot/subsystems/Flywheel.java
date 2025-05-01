// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.ShooterConstants.SHOOTER_SPEEDS;

public class Flywheel extends SubsystemBase {
	private ThunderBird topMotor;
	private ThunderBird bottomMotor;

	private TalonFXConfiguration config = new TalonFXConfiguration();
	private VelocityVoltage velocityPID;

	private double targetRPM = 0;

	private boolean coast = false;

	public Flywheel() {
		topMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD, false, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);
		bottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD, false, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT, false);

		config.Slot0.kP = LightningShuffleboard.getDouble("Flywheel", "kP", FlywheelConstants.TOP_0_MOTOR_KP);
		config.Slot0.kV = LightningShuffleboard.getDouble("Flywheel", "kV", FlywheelConstants.TOP_0_MOTOR_KV);
		config.Slot0.kD = FlywheelConstants.TOP_0_MOTOR_KD;
		config.Slot0.kV = FlywheelConstants.TOP_0_MOTOR_KV;

		topMotor.applyConfig(config);
		velocityPID = new VelocityVoltage(0).withSlot(0);

		bottomMotor.setControl(new Follower(RobotMap.CAN.FLYWHEEL_MOTOR_TOP, false));
	}

	@Override
	public void periodic() {
		LightningShuffleboard.setDouble("Flywheel", "top motor speed", topMotor.getVelocity().getValueAsDouble());
		LightningShuffleboard.setDouble("Flywheel", "bottom motor speed", bottomMotor.getVelocity().getValueAsDouble());

		targetRPM = coast ? FlywheelConstants.COAST_RPM : targetRPM;
		topMotor.setControl(velocityPID.withVelocity(targetRPM));
	}

	public void setTargetRPM(double newTargetRPM){
		targetRPM = newTargetRPM;
	}

	public boolean onTarget(){
		return Math.abs(topMotor.getVelocity().getValueAsDouble() - targetRPM) < FlywheelConstants.RPM_TOLERANCE;
	}

	public void startCoast(){
		coast = true;
	}

	public void stopCoast(){
		coast = false;
	}

	public void stop(){
		targetRPM = 0;
	}
}

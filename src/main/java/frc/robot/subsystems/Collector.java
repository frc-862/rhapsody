package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.config.FalconConfig;

public class Collector extends SubsystemBase {

	// Declare collector hardware
	private TalonFX collectorMotorFront;
	private TalonFX collectorMotorBack;
	private DigitalInput collectorEntryBeamBreakFront;
	private DigitalInput collectorEntryBeamBreakBack;

	public Collector() {
		// Initialize collector hardware
		collectorMotorFront = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR_FRONT, getName(),
				CollectorConstants.COLLECTOR_MOTOR_INVERTED_FRONT,
				CollectorConstants.COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_FRONT,
				CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_FRONT,
				CollectorConstants.COLLECTOR_MOTOR_NEUTRAL_MODE_FRONT);
		collectorMotorBack = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR_BACK, getName(),
				CollectorConstants.COLLECTOR_MOTOR_INVERTED_BACK,
				CollectorConstants.COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_BACK,
				CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_BACK,
				CollectorConstants.COLLECTOR_MOTOR_NEUTRAL_MODE_BACK);
		collectorEntryBeamBreakFront = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_FRONT);
		collectorEntryBeamBreakBack = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_BACK);
	}

	@Override
	public void periodic() {

	}

	/**
	 * Front Collector Beam Break
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getFrontEntryBeamBreakState() {
		return !collectorEntryBeamBreakFront.get();
	}

	/**
	 * Back Collector Beam Break
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getBackEntryBeamBreakState() {
		return !collectorEntryBeamBreakBack.get();
	}


	/**
	 * Sets the power of both collector motors
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPower(double power) {
		collectorMotorFront.set(power);
		collectorMotorBack.set(power);
	}
	
	/**
	 * Sets the power of the Front collector motor
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPowerFront(double power) {
		collectorMotorFront.set(power);
	}

	/**
	 * Sets the power of the Back collector motor
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPowerBack(double power) {
		collectorMotorBack.set(power);
	}

	/**
	 * Stops the collector
	 */
	public void stop() {
		setPower(0d);
	}

	
}

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
	private TalonFX collectorMotorTop;
	private TalonFX collectorMotorBottom;
	private DigitalInput collectorEntryBeamBreakEntrance;
	private DigitalInput collectorEntryBeamBreakExit;

	public Collector() {
		// Initialize collector hardware
		collectorMotorTop = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR_TOP, CAN.CANBUS_FD,
				CollectorConstants.COLLECTOR_MOTOR_INVERTED_TOP,
				CollectorConstants.COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_TOP,
				CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_TOP,
				CollectorConstants.COLLECTOR_MOTOR_NEUTRAL_MODE_TOP);
		collectorMotorBottom = FalconConfig.createMotor(CAN.COLLECTOR_MOTOR_BOTTOM, CAN.CANBUS_FD,
				CollectorConstants.COLLECTOR_MOTOR_INVERTED_BOTTOM,
				CollectorConstants.COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_BOTTOM,
				CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_BOTTOM,
				CollectorConstants.COLLECTOR_MOTOR_NEUTRAL_MODE_BOTTOM);
		collectorEntryBeamBreakEntrance = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_FRONT);
		collectorEntryBeamBreakExit = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_BACK);
		collectorEntryBeamBreakExit = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_BACK);
	}

	@Override
	public void periodic() {

	}

	/**
	 * Entrance of Collector Beam Break
	 * 
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getEntryBeamBreakState() {
		return !collectorEntryBeamBreakEntrance.get();
	}

	/**
	 * Exit of Collector Beam Break
	 * 
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getExitBeamBreakState() {
		return !collectorEntryBeamBreakExit.get();
	}


	/**
	 * Sets the power of both collector motors
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPower(double power) {
		collectorMotorTop.set(power);
		collectorMotorBottom.set(power);
	}

	/**
	 * Sets the power of the top collector motor
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPowerTop(double power) {
		collectorMotorTop.set(power);
	}

	/**
	 * Sets the power of the bottom collector motor
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPowerBottom(double power) {
		collectorMotorBottom.set(power);
	}

	/**
	 * Has piece
	 */
	public boolean hasPiece() {
		// Could use beam breaks and store when a piece enters until it leaves
		return false; // TODO add actual logic
	}

	/**
	 * Stops the collector
	 */
	public void stop() {
		setPower(0d);
	}

}

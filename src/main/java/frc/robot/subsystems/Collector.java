package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collector extends SubsystemBase {

	// Declare collector hardware
	private ThunderBird motor;
	private DigitalInput beamBreak;

	private boolean hasPiece;

	public Collector() {
		motor = new ThunderBird(
				CAN.COLLECTOR_MOTOR, CAN.CANBUS_FD,
				CollectorConstants.COLLECTOR_MOTOR_INVERTED,
				CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT,
				CollectorConstants.COLLECTOR_MOTOR_BRAKE_MODE);

		beamBreak = new DigitalInput(DIO.COLLECTOR_BEAMBREAK);
		initLogging();
	}

	private void initLogging() {
		LightningShuffleboard.setDoubleSupplier("Collector", "Collector Power", () -> motor.get());
		LightningShuffleboard.setBoolSupplier("Collector", "Beam Break", () -> getEntryBeamBreakState());
		LightningShuffleboard.setBoolSupplier("Collector", "Has Piece", () -> hasPiece());
	}

	/**
	 * Entrance of Collector Beam Break
	 * 
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getEntryBeamBreakState() {
		return !beamBreak.get();
	}

	/**
	 * Sets the power of both collector motors
	 * 
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPower(double power) {
		motor.set(power);
	}

	@Override
	public void periodic() {
		// tells robot if we have a piece in collector
		hasPiece = getEntryBeamBreakState();
	}

	/**
	 * Has piece
	 * 
	 * @return boolean, true if collector has piece
	 */
	public boolean hasPiece() {
		return hasPiece;
	}

	/**
	 * Stops the collector
	 */
	public void stop() {
		setPower(0d);
	}
}

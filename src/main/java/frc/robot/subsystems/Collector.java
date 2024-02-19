package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.thunder.hardware.ThunderBird;

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
		// TODO: Could use beam breaks and store when a piece enters until it leaves
		return hasPiece;
	}

	/**
	 * Stops the collector
	 */
	public void stop() {
		setPower(0d);
	}
}

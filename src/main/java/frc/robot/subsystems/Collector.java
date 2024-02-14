package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.hardware.ThunderBird;

public class Collector extends SubsystemBase {

	// Declare collector hardware
	private ThunderBird motor;
	private DigitalInput collectorEntryBeamBreakEntrance;
	private DigitalInput collectorEntryBeamBreakExit;
	private boolean hasPiece;

	public Collector() {
		motor = new ThunderBird(CAN.COLLECTOR_MOTOR, CAN.CANBUS_FD,
				CollectorConstants.COLLECTOR_MOTOR_INVERTED, CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT, 
				CollectorConstants.COLLECTOR_MOTOR_BRAKE_MODE);

		//TODO: real robot doesn't currently have a plan to have 2 beam breaks, if programming requires this they need to say so soon.
		collectorEntryBeamBreakEntrance = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_FRONT);
		collectorEntryBeamBreakExit = new DigitalInput(DIO.COLLECTOR_ENTRY_BEAMBREAK_BACK);

	}

	/**
	 * Entrance of Collector Beam Break
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getEntryBeamBreakState() {
		return !collectorEntryBeamBreakEntrance.get();
	}

	/**
	 * Exit of Collector Beam Break
	 * @return When an object is present, returns true, otherwise returns false
	 */
	public boolean getExitBeamBreakState() {
		return !collectorEntryBeamBreakExit.get();
	}

	/**
	 * Sets the power of both collector motors
	 * @param power Double value from -1.0 to 1.0 (positive collects inwards)
	 */
	public void setPower(double power) {
		motor.set(power);
	}

	@Override
	public void periodic() {
		// tells robot if we have a piece in collector
		if (getEntryBeamBreakState()) {
			hasPiece = true;
		} else if (getExitBeamBreakState()){
			hasPiece = false;
		}
	}

	/**
	 * Has piece
	 * @return boolean, true if collector has piece
	 */
	public boolean hasPiece() {
		// Could use beam breaks and store when a piece enters until it leaves
		return hasPiece;
	}

	/**
	 * Stops the collector
	 */
	public void stop() {
		setPower(0d);
	}
}

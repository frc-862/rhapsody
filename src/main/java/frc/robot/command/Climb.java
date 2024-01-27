package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climber;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.subsystems.Swerve;
import frc.robot.command.TipDetection;

public class Climb extends Command {
	// create vars
	private PIDController climbController = new PIDController(0.5, 0, 0); // TODO: tune pid
	private Climber climber;
	private double setPoint;
	private TipDetection tipDetection;

	/**
	 * Creates a new Climb.
	 * 
	 * @param climber subsystem
	 * @param drivetrain subsystem
	 */
	public Climb(Climber climber, Swerve drivetrain) {
		this.climber = climber;
		this.tipDetection = new TipDetection(drivetrain);

		addRequirements(climber);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		setPoint = ClimbConstants.CLIMB_PID_SETPOINT_EXTENDED;
	}

	/**
	 * retracts climb to 0
	 */
	public void retract() {
		setPoint = ClimbConstants.CLIMB_PID_SETPOINT_RETRACTED;
	}

	/**
	 * @return returns whether climb is extended
	 */
	public boolean extended() {
		return (setPoint - climber.getHeight() < ClimbConstants.CLIMB_EXTENSION_TOLERANCE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// use pid to set climber power
		double pidOutput = climbController.calculate(climber.getHeight(), setPoint);
		climber.setPower(pidOutput);

		if (extended() && tipDetection.isTipped()) {
			retract();
		}

		LightningShuffleboard.setDouble("Climb", "Climb Height", climber.getHeight());
    	LightningShuffleboard.setDouble("Climb", "Climb Power", pidOutput);
	}

	/**
	 * @return climb setpoint
	 */
	public double getSetPoint() {
		return setPoint;
	}

	/**
	 * set setpoint for climb pid
	 * @param setPoint height to go to in TODO pick a unit
	 */
	public void setSetPoint(double setPoint) {
		this.setPoint = setPoint;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		climber.stopClimb();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

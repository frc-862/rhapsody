package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector;
import frc.robot.command.Collect;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.DrivetrAinConstants;

import frc.thunder.LightningContainer;

public class RobotContainer extends LightningContainer {
	/* Setting up bindings for necessary control of the swerve drive platform */
	XboxController driver;
	XboxController coPilot;

	private Swerve drivetrain;
	// Collector collector = new Collector();
	// Flywheel flywheel = new Flywheel();
	// Pivot pivot = new Pivot();
	// Shooter shooter = new Shooter(pivot, flywheel);


	// TODO I want field-centric driving in open loop WE NEED TO FIGURE OUT WHAT
	// Change beacuse with open loop is gone
	SwerveRequest.FieldCentric drive;
	SwerveRequest.SwerveDriveBrake brake;
	SwerveRequest.PointWheelsAt point;
	// Telemetry logger;

	@Override
	protected void initializeSubsystems() {
		driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
		coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller

		drivetrain = TunerConstants.getDrivetrain(); // My drivetrain
		// collector = new Collector();
		// flywheel = new Flywheel();
		// pivot = new Pivot();
		// shooter = new Shooter(pivot, flywheel);

		// TODO I want field-centric driving in open loop WE NEED TO FIGURE OUT WHAT
		// Change beacuse with open loop is gone
		drive = new SwerveRequest.FieldCentric().withDeadband(DrivetrAinConstants.MaxSpeed * 0.1)
				.withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * 0.1) // Add a 10% deadband
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

		brake = new SwerveRequest.SwerveDriveBrake();
		point = new SwerveRequest.PointWheelsAt();
		// logger = new Telemetry(DrivetrAinConstants.MaxSpeed);
	}

	@Override
	protected void configureButtonBindings() {
		new Trigger(driver::getLeftBumper).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
	}

	@Override
	protected void configureDefaultCommands() {
		// drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * DrivetrAinConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
						.withVelocityY(-driver.getLeftX() * DrivetrAinConstants.MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-driver.getRightX() * DrivetrAinConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
				));

		// collector.setDefaultCommand(new Collect(() -> (coPilot.getRightTriggerAxis()
		// - coPilot.getLeftTriggerAxis()), collector));
	}

	@Override
	protected void configureAutonomousCommands() {

	}

	@Override
	protected void releaseDefaultCommands() {
	}

	@Override
	protected void initializeDashboardCommands() {
	}

	@Override
	protected void configureFaultCodes() {
	}

	@Override
	protected void configureFaultMonitors() {
	}

	@Override
	protected void configureSystemTests() {
	}
}

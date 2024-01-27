package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Collector;
import frc.robot.command.TipDetection;
import frc.robot.command.chasePieces;
import frc.robot.command.PointAtTag;
import frc.robot.command.Collect;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrAinConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.command.PointAtTag;
import frc.robot.command.Shoot;
import frc.robot.command.Climb;
import frc.robot.command.ManualClimb;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class RobotContainer extends LightningContainer {
	XboxController driver;
	XboxController coPilot;

	//Subsystems
	private Swerve drivetrain;
	// Collector collector;
	// Flywheel flywheel;
	// Pivot pivot;
	// Shooter shooter;
	// Collision collision;
	// Indexer indexer;
	// Climber climber;

	private SendableChooser<Command> autoChooser;
	// TODO I want field-centric driving in open loop WE NEED TO FIGURE OUT WHAT
	// Change beacuse with open loop is gone
	SwerveRequest.FieldCentric drive;
	SwerveRequest.FieldCentric slow;
	SwerveRequest.SwerveDriveBrake brake;
	SwerveRequest.PointWheelsAt point;
	Telemetry logger;

	@Override
	protected void initializeSubsystems() {
		driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
		coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller
		
		drivetrain = TunerConstants.getDrivetrain(); // My drivetrain
		
		autoChooser = AutoBuilder.buildAutoChooser();	
		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
		
		// indexer = new Indexer();
		// collector = new Collector();
		// flywheel = new Flywheel();
		// pivot = new Pivot();
		// shooter = new Shooter(pivot, flywheel, indexer);
		// collision = new Collision(drivetrain);
		// climber = new Climber();

		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
		slow = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop

		brake = new SwerveRequest.SwerveDriveBrake();
		point = new SwerveRequest.PointWheelsAt();
		logger = new Telemetry(DrivetrAinConstants.MaxSpeed);
	}

	@Override
	protected void configureButtonBindings() { //TODO decide on comp buttons
		new Trigger(driver::getLeftBumper).onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(driver::getRightBumper).whileTrue(
			drivetrain.applyRequest(() -> slow.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
				.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.SLOW_ROT_MULT))); // Drive counterclockwise with negative X (left)

		new Trigger(driver::getRightBumper).onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true))).onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));
	
		new Trigger(driver::getXButton).whileTrue(new chasePieces(drivetrain));
		new Trigger(driver::getBackButton).whileTrue(new TipDetection(drivetrain));


		// new Trigger(driver::getYButton).whileTrue(new Climb(climber, drivetrain));
	}

	@Override
	protected void initializeNamedCommands() {
	}

	@Override
	protected void configureDefaultCommands() {
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
						.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND) * DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_MULT) // Drive counterclockwise with negative X (left)
				));
		// climber.setDefaultCommand(new ManualClimb(() -> (coPilot.getRightTriggerAxis() - coPilot.getLeftTriggerAxis()), climber));

		// shooter.setDefaultCommand(new Shoot(shooter, indexer, drivetrain, () -> coPilot.getAButton()));

		// collector.setDefaultCommand(new Collect(() -> (coPilot.getRightTriggerAxis()
		// - coPilot.getLeftTriggerAxis()), collector));
	}

	protected Command getAutonomousCommand(){
		return autoChooser.getSelected();
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

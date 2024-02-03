package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants.CAND_STATES;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.command.ChasePieces;
import frc.robot.command.Index;
import frc.robot.command.PointAtTag;
import frc.robot.command.SetLEDState;
import frc.robot.command.TipDetection;
import frc.robot.command.tests.DrivetrainSystemTest;
import frc.robot.command.tests.TurnSystemTest;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Collector;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {
	public XboxController driver;
	public XboxController coPilot;

	//Subsystems
	private Swerve drivetrain;
	Collector collector;
	Flywheel flywheel;
	Pivot pivot;
	Shooter shooter;
	// Collision collision;
	Indexer indexer;
	// Climber climber;
	LEDs leds;

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
		SignalLogger.setPath(Constants.HOOT_PATH);
		SignalLogger.enableAutoLogging(true);

		driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
		coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller
		
		drivetrain = TunerConstants.getDrivetrain(); // My drivetrain
		
		indexer = new Indexer();
		collector = new Collector();
		flywheel = new Flywheel();
		pivot = new Pivot();
		shooter = new Shooter(pivot, flywheel, indexer, collector);
		// collision = new Collision(drivetrain);
		// climber = new Climber();
		leds = new LEDs();

		drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop
		slow = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop

		brake = new SwerveRequest.SwerveDriveBrake();
		point = new SwerveRequest.PointWheelsAt();
		logger = new Telemetry(DrivetrainConstants.MaxSpeed);
	}

	@Override
	protected void initializeNamedCommands() {
		NamedCommands.registerCommand("disable-Vision", new InstantCommand(() -> drivetrain.disableVision()));
		NamedCommands.registerCommand("enable-Vision", new InstantCommand(() -> drivetrain.enableVision()));

		// make sure named commands is initialized before autobuilder!
		autoChooser = AutoBuilder.buildAutoChooser();	
		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
	}

	@Override
	protected void configureButtonBindings() {
		/* driver */
		new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(() -> driver.getRightTriggerAxis() > 0.25d).whileTrue(
			drivetrain.applyRequest(() -> slow.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
				.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
				.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate * DrivetrainConstants.SLOW_ROT_MULT))); // Drive counterclockwise with negative X (left)
		new Trigger(() -> driver.getRightTriggerAxis() > 0.25d).onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true))).onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));
		new Trigger(driver::getXButton).whileTrue(new ChasePieces(drivetrain));

		/* copilot */
		// new Trigger(coPilot::getBButton).whileTrue(new InstantCommand(() -> shooter.setState(SHOOTER_STATES.STOW)));
		// new Trigger(coPilot::getRightBumper).whileTrue(new Index(indexer,() -> IndexerConstants.INDEXER_DEFAULT_POWER));
		// new Trigger(coPilot::getLeftBumper).whileTrue(new Index(indexer,() -> -IndexerConstants.INDEXER_DEFAULT_POWER));
		new Trigger(() -> coPilot.getAButton() || coPilot.getBButton() || coPilot.getXButton()).whileTrue(new InstantCommand(() -> shooter.setState(SHOOTER_STATES.CAND_SHOTS)));
		new Trigger(coPilot::getAButton).whileTrue(new InstantCommand(() -> shooter.setCANDState(CAND_STATES.AMP)));
		new Trigger(coPilot::getBButton).whileTrue(new InstantCommand(() -> shooter.setCANDState(CAND_STATES.SUBWOOFER)));
		new Trigger(coPilot::getXButton).whileTrue(new InstantCommand(() -> shooter.setCANDState(CAND_STATES.PODIUM)));
	}

	@Override
	protected void configureDefaultCommands() {
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis_
						.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT) // Drive counterclockwise with negative X (left)
				));
		// climber.setDefaultCommand(new ManualClimb(() -> (coPilot.getRightTriggerAxis() - coPilot.getLeftTriggerAxis()), climber));
		// climber.setDefaultCommand(new Climb(climber, ClimbConstants.CLIMB_PID_SETPOINT_RETRACTED));

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
		SystemTest.registerTest("Drive Test", new DrivetrainSystemTest(drivetrain, brake, DrivetrainConstants.SYS_TEST_SPEED_DRIVE));
		SystemTest.registerTest("Azimuth Test", new TurnSystemTest(drivetrain, brake, DrivetrainConstants.SYS_TEST_SPEED_TURN));

		// SystemTest.registerTest("Shooter Test", new ShooterSystemTest(shooter, flywheel, collector, indexer, pivot));
	}
}

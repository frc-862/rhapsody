package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MusicConstants;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.command.ChasePieces;
import frc.robot.command.Index;
import frc.robot.command.MoveToPose;
import frc.robot.command.ManualClimb;
import frc.robot.command.PointAtTag;
import frc.robot.command.Sing;
import frc.robot.command.shoot.AmpShot;
import frc.robot.command.shoot.PodiumShot;
import frc.robot.command.shoot.PointBlankShot;
import frc.robot.command.shoot.SmartShoot;
import frc.robot.command.tests.DrivetrainSystemTest;
import frc.robot.command.tests.SingSystemTest;
import frc.robot.command.tests.TurnSystemTest;
import frc.robot.command.Climb;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.thunder.LightningContainer;
import frc.thunder.command.TimedCommand;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.testing.SystemTest;
import frc.thunder.testing.SystemTestCommand;

public class RobotContainer extends LightningContainer {
	public static XboxController driver;
	public static XboxController coPilot;

	// Subsystems
	private Swerve drivetrain;
	private Limelights limelights;
	// Collector collector;
	// Flywheel flywheel;
	// Pivot pivot;
	// Shooter shooter;
	// Indexer indexer;
	// Climber climber;
	LEDs leds;
	Orchestra sing;

	private SendableChooser<Command> autoChooser;
	SwerveRequest.FieldCentric drive;
	SwerveRequest.FieldCentric slow;
	SwerveRequest.RobotCentric driveRobotCentric;
	SwerveRequest.RobotCentric slowRobotCentric;
	SwerveRequest.SwerveDriveBrake brake;
	SwerveRequest.PointWheelsAt point;
	Telemetry logger;

	@Override
	protected void initializeSubsystems() {
		SignalLogger.setPath(Constants.HOOT_PATH);
		SignalLogger.enableAutoLogging(true);

		driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
		coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller
		
		limelights = new Limelights();
		drivetrain = TunerConstants.getDrivetrain(limelights);

		// indexer = new Indexer();
		// collector = new Collector();
		// flywheel = new Flywheel();
		// pivot = new Pivot();
		// shooter = new Shooter(pivot, flywheel, indexer, collector);
		// climber = new Climber(drivetrain);
		leds = new LEDs();
		sing = new Orchestra();

		// field centric for the robot
		drive = new SwerveRequest.FieldCentric()
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);// .withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop

		// robot centric for the robot
		driveRobotCentric = new SwerveRequest.RobotCentric()
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);// .withDeadband(DrivetrAinConstants.MaxSpeed * DrivetrAinConstants.SPEED_DB).withRotationalDeadband(DrivetrAinConstants.MaxAngularRate * DrivetrAinConstants.ROT_DB); // I want field-centric driving in closed loop

		brake = new SwerveRequest.SwerveDriveBrake();
		point = new SwerveRequest.PointWheelsAt();
		logger = new Telemetry(DrivetrainConstants.MaxSpeed);

	}

	@Override
	protected void initializeNamedCommands() {
		NamedCommands.registerCommand("disable-Vision",
				new InstantCommand(() -> drivetrain.disableVision()));
		NamedCommands.registerCommand("enable-Vision",
				new InstantCommand(() -> drivetrain.enableVision()));
		NamedCommands.registerCommand("led-Collect",
				leds.enableState(LED_STATES.COLLECTED).withTimeout(0.5));
		NamedCommands.registerCommand("led-Shoot",
				leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5));

		// make sure named commands is initialized before autobuilder!
		autoChooser = AutoBuilder.buildAutoChooser();
		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);

		
	}
	@Override
	protected void configureButtonBindings() {
		/* driver */
		// activates between field centric and robot centric drive + logs
		new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(new InstantCommand(() -> drivetrain.setRobotCentricControl(true))).whileTrue(
			// robot centric and slow mode
			drivetrain.inSlowMode() ?
				drivetrain.applyRequest(() -> driveRobotCentric
					.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
					.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
					.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
						* DrivetrainConstants.SLOW_ROT_MULT)) // Drive counterclockwise with negative X (left)
			// only robot centric drive
			:
				drivetrain.applyRequest(() -> driveRobotCentric
					.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
					.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive left with negative X (left)
					.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
						* DrivetrainConstants.SLOW_ROT_MULT))) // Drive counterclockwise with negative X (left)
		.whileFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

		// activates between regular speed and slow mode + logs
		new Trigger(() -> driver.getRightTriggerAxis() > 0.25d).whileTrue(new InstantCommand(() -> drivetrain.setSlowMode(true))).whileTrue(
			// slow mode and robot centric
			drivetrain.isRobotCentricControl() ?
				drivetrain.applyRequest(() -> driveRobotCentric
					.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
					.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
					.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
						* DrivetrainConstants.SLOW_ROT_MULT)) // Drive counterclockwise with negative X (left)
			// only slow mode
			:
				drivetrain.applyRequest(() -> drive
					.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
					.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT) // Drive left with negative X (left)
					.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
						ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
						* DrivetrainConstants.SLOW_ROT_MULT))) // Drive counterclockwise with negative X (left)
		.whileFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

		// resets the gyro of the robot
		new Trigger(() -> driver.getStartButton() && driver.getBackButton())
				.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));
		
		// makes the robot chase pieces
		// new Trigger(driver::getRightBumper).whileTrue(new ChasePieces(drivetrain, collector, limelights));

		// parks the robot
		new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(() -> brake));
		
		// smart shoot for the robot
		// new Trigger(driver::getAButton).whileTrue(new SmartShoot(flywheel, pivot, drivetrain, indexer, leds));

		// aim at amp and stage tags for the robot
		new Trigger(driver::getLeftBumper).whileTrue(new PointAtTag(drivetrain, limelights, driver)); // TODO: make work

		new Trigger(driver::getYButton).whileTrue(new MoveToPose(AutonomousConstants.TARGET_POSE, drivetrain, drive).alongWith(hapticDriverCommand()));
		
		new Trigger(() -> driver.getPOV() == 0).toggleOnTrue(leds.enableState(LED_STATES.DISABLED));

		/* copilot */
		// cand shots for the robot
		// new Trigger(coPilot::getAButton).whileTrue(new AmpShot(flywheel, pivot));
		// new Trigger(coPilot::getXButton).whileTrue(new PointBlankShot(flywheel, pivot));
		// new Trigger(coPilot::getYButton).whileTrue(new PodiumShot(flywheel, pivot));

		// new Trigger(coPilot::getBButton).whileTrue(new Climb(climber, drivetrain).deadlineWith(leds.enableState(LED_STATES.CLIMBING)));

		/*BIAS */
		// new Trigger(() -> coPilot.getPOV() == 0).onTrue(new InstantCommand(() -> pivot.increaseBias())); // UP
		// new Trigger(() -> coPilot.getPOV() == 180).onTrue(new InstantCommand(() -> pivot.decreaseBias())); // DOWN

		// new Trigger(() -> coPilot.getPOV() == 90).onTrue(new InstantCommand(() -> flywheel.increaseBias())); // RIGHT
		// new Trigger(() -> coPilot.getPOV() == 270).onTrue(new InstantCommand(() -> flywheel.decreaseBias())); // LEFT

		// new Trigger(coPilot::getRightBumper).whileTrue(new Index(indexer,() -> IndexerConstants.INDEXER_DEFAULT_POWER));
		// new Trigger(coPilot::getLeftBumper).whileTrue(new Index(indexer,() -> -IndexerConstants.INDEXER_DEFAULT_POWER));

		/* Other */
		new Trigger(() -> (limelights.getStopMe().hasTarget() || limelights.getChamps().hasTarget())).whileTrue(leds.enableState(LED_STATES.HAS_VISION));
		// new Trigger(() -> collector.hasPiece()).whileTrue(leds.enableState(LED_STATES.HAS_PIECE).withTimeout(2)).onTrue(leds.enableState(LED_STATES.COLLECTED).withTimeout(2));

	}

	

	@Override
	protected void configureDefaultCommands() {
		/* driver */
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(),
								ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive forward with negative Y (Its worth noting the field Y axis differs from the robot Y axis
						.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(),
								ControllerConstants.DEADBAND) * DrivetrainConstants.MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(),
								ControllerConstants.DEADBAND) * DrivetrainConstants.MaxAngularRate
								* DrivetrainConstants.ROT_MULT) // Drive counterclockwise with negative X (left)
				));


		/* copilot */
		// collector.setDefaultCommand(new Collect(() -> (coPilot.getRightTriggerAxis() -
		// coPilot.getLeftTriggerAxis()), collector));

		// climber.setDefaultCommand(new ManualClimb(() -> coPilot.getLeftY(),() ->
		// coPilot.getRightY(), climber));
	}

	protected Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	@Override
	protected void releaseDefaultCommands() {}

	@Override
	protected void initializeDashboardCommands() {}

	@Override
	protected void configureFaultCodes() {}

	@Override
	protected void configureFaultMonitors() {}

	@Override
	protected void configureSystemTests() {
		SystemTest.registerTest("Drive Test", new DrivetrainSystemTest(drivetrain, brake,
				DrivetrainConstants.SYS_TEST_SPEED_DRIVE));
		SystemTest.registerTest("Azimuth Test",
				new TurnSystemTest(drivetrain, brake, DrivetrainConstants.SYS_TEST_SPEED_TURN));

		// SystemTest.registerTest("Shooter Test", new ShooterSystemTest(shooter, flywheel,
		// collector, indexer, pivot));
		
		//Make sing appear on shuffleboard!! :)
		SendableChooser<SystemTestCommand> songChooser = new SendableChooser<>();
		songChooser.setDefaultOption(MusicConstants.BOH_RHAP_FILEPATH, new SingSystemTest(drivetrain, MusicConstants.BOH_RHAP_FILEPATH, sing));
		for (String filepath: MusicConstants.SET_LIST){
			songChooser.addOption(filepath, new SingSystemTest(drivetrain, filepath, sing));
		}

		LightningShuffleboard.set("SystemTest", "Songs List", songChooser);
		
		songChooser.onChange((SystemTestCommand command) -> {
			if (command != null) {
				command.schedule();
			}
		});


		songChooser.close();

	}

	public static Command hapticDriverCommand() {
		return new StartEndCommand(() -> {
			driver.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
			driver.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
		}, () -> {
			driver.setRumble(GenericHID.RumbleType.kRightRumble, 0);
			driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
		});
	}

	public static Command hapticCopilotCommand() {
		return new StartEndCommand(() -> {
			coPilot.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
			coPilot.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
		}, () -> {
			coPilot.setRumble(GenericHID.RumbleType.kRightRumble, 0);
			coPilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
		});
	}
}

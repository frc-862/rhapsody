package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Constants.CollisionConstants.CollisionType;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.command.ChasePieces;
import frc.robot.command.Index;
import frc.robot.command.MoveToPose;
import frc.robot.command.PointAtPoint;
import frc.robot.command.ManualClimb;
import frc.robot.command.PointAtTag;
import frc.robot.command.Sing;
import frc.robot.command.SmartCollect;
import frc.robot.command.shoot.AmpShot;
import frc.robot.command.shoot.PodiumShot;
import frc.robot.command.shoot.PointBlankShot;
import frc.robot.command.shoot.SmartShoot;
import frc.robot.command.shoot.SourceCollect;
import frc.robot.command.shoot.Stow;
import frc.robot.command.shoot.Tune;
import frc.robot.command.shoot.AutonCand.AmpShotAuton;
import frc.robot.command.shoot.AutonCand.CandC1;
import frc.robot.command.shoot.AutonCand.CandC2;
import frc.robot.command.shoot.AutonCand.CandC3;
import frc.robot.command.shoot.AutonCand.CandLine;
import frc.robot.command.shoot.AutonCand.PointBlankShotAuton;
import frc.robot.command.tests.ClimbSystemTest;
import frc.robot.command.tests.CollectorSystemTest;
import frc.robot.command.tests.DrivetrainSystemTest;
import frc.robot.command.tests.FlywheelSystemTest;
import frc.robot.command.tests.SingSystemTest;
import frc.robot.command.tests.TurnSystemTest;
import frc.robot.command.Climb;
import frc.robot.command.CollisionDetection;
import frc.robot.command.Collect;
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
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.testing.SystemTest;
import frc.thunder.testing.SystemTestCommand;

public class RobotContainer extends LightningContainer {
	public static XboxControllerFilter driver;
	public static XboxControllerFilter coPilot;

	// Subsystems
	private Swerve drivetrain;
	private Limelights limelights;
	private Collector collector;
	private Flywheel flywheel;
	private Pivot pivot;
	private Indexer indexer;
	// private Climber climber;
	LEDs leds;
	Orchestra sing;

	private SendableChooser<Command> autoChooser;
	SwerveRequest.FieldCentric drive;
	SwerveRequest.FieldCentric slow;
	SwerveRequest.RobotCentric driveRobotCentric;
	SwerveRequest.RobotCentric slowRobotCentric;
	SwerveRequest.PointWheelsAt point;
	Telemetry logger;

	@Override
	protected void initializeSubsystems() {
		// SignalLogger.setPath(Constants.HOOT_PATH);
		// SignalLogger.enableAutoLogging(true);

		driver = new XboxControllerFilter(ControllerConstants.DriverControllerPort,
		Constants.ControllerConstants.DEADBAND, -1, 1,
		XboxControllerFilter.filterMode.SQUARED); // Driver controller
		coPilot = new XboxControllerFilter(ControllerConstants.CopilotControllerPort,
		Constants.ControllerConstants.DEADBAND, -1, 1,
		XboxControllerFilter.filterMode.SQUARED); // CoPilot controller


		limelights = new Limelights();
		drivetrain = TunerConstants.getDrivetrain(limelights);

		collector = new Collector();
		flywheel = new Flywheel();
		pivot = new Pivot();
		indexer = new Indexer(collector);
		// climber = new Climber(drivetrain);
		leds = new LEDs();
		sing = new Orchestra();

		point = new SwerveRequest.PointWheelsAt();
		logger = new Telemetry(DrivetrainConstants.MaxSpeed);
	}

	@Override
	protected void initializeNamedCommands() {
		NamedCommands.registerCommand("disable-Vision",
				new InstantCommand(() -> drivetrain.disableVision()));
		NamedCommands.registerCommand("enable-Vision",
				new InstantCommand(() -> drivetrain.enableVision()));
		NamedCommands.registerCommand("led-Shoot",
				leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5));

		NamedCommands.registerCommand("Cand-Sub", 
			new PointBlankShotAuton(flywheel, pivot, indexer)
				.alongWith(leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5))); 
		NamedCommands.registerCommand("Cand-C1", new CandC1(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-C2", new CandC2(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-C3", new CandC3(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-Line", new CandLine(flywheel, pivot, indexer));
		NamedCommands.registerCommand("AMP", new AmpShotAuton(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Stow", new Stow(flywheel, pivot));
		NamedCommands.registerCommand("Smart-Shoot",
			new SmartShoot(flywheel, pivot, drivetrain, indexer, leds)
				.alongWith(leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5)));
		NamedCommands.registerCommand("Chase-Pieces", new ChasePieces(drivetrain, collector, indexer, pivot, limelights));
		NamedCommands.registerCommand("Collect",
			new SmartCollect(() -> .5d, () -> .6d, collector, indexer, pivot)
				.alongWith(leds.enableState(LED_STATES.COLLECTING).withTimeout(1)));
		NamedCommands.registerCommand("Index-Up", new Index(indexer, () -> IndexerConstants.INDEXER_DEFAULT_POWER));
		NamedCommands.registerCommand("PathFind", new MoveToPose(AutonomousConstants.TARGET_POSE, drivetrain));

		// make sure named commands are initialized before autobuilder!
		autoChooser = AutoBuilder.buildAutoChooser();
		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
	}

	@Override
	protected void configureButtonBindings() {
		/* driver */
		// field centric for the robot
		new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d)
				.onTrue(new InstantCommand(() -> drivetrain.setRobotCentricControl(true)))
				.whileTrue(drivetrain.applyPercentRequestRobot(
						() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()))
				.onFalse(new InstantCommand(() -> drivetrain.setRobotCentricControl(false)));

		// enables slow mode for driving
		new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
				.onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)))
				.onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

		// sets field relative forward to the direction the robot is facing
		new Trigger(() -> driver.getStartButton() && driver.getBackButton())
				.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

		// makes the robot chase pieces
		new Trigger(driver::getRightBumper).whileTrue(new ChasePieces(drivetrain, collector, indexer, pivot, limelights)
			.deadlineWith(leds.enableState(LED_STATES.CHASING)));

		// parks the robot
		// new Trigger(driver::getXButton).whileTrue(new InstantCommand(() -> drivetrain.brake()));

		// smart shoot for the robot
		new Trigger(driver::getAButton)
			.whileTrue(new SmartShoot(flywheel, pivot, drivetrain, indexer, leds)
			.alongWith(leds.enableState(LED_STATES.SHOOTING)));

		// aim at amp and stage tags for the robot
		new Trigger(driver::getXButton)
				.whileTrue(new PointAtTag(drivetrain, limelights, driver)); // TODO: make work
		
		new Trigger(driver::getLeftBumper).whileTrue(new PointAtPoint(VisionConstants.SPEAKER_LOCATION.getX(), VisionConstants.SPEAKER_LOCATION.getY(), drivetrain, driver));

		// new Trigger(driver::getYButton)
		// .whileTrue(new MoveToPose(AutonomousConstants.TARGET_POSE, drivetrain));

		new Trigger(() -> driver.getPOV() == 0).toggleOnTrue(leds.enableState(LED_STATES.DISABLED));

		/* copilot */
		new Trigger(coPilot::getBButton)
		.whileTrue(new SmartCollect(() -> 0.50, () -> 0.60, collector, indexer, pivot)); // TODO: find correct button/trigger

		// cand shots for the robot
		new Trigger(coPilot::getAButton).whileTrue(new AmpShot(flywheel, pivot));
		new Trigger(coPilot::getXButton).whileTrue(new PointBlankShot(flywheel, pivot));
		// new Trigger(coPilot::getXButton).whileTrue(new Tune(flywheel, pivot));
		// new Trigger(coPilot::getYButton).whileTrue(new PodiumShot(flywheel, pivot));
		new Trigger(coPilot::getYButton).whileTrue(new SourceCollect(flywheel, pivot));

		// new Trigger(coPilot::getBButton).whileTrue(new Climb(climber,  // TODO need new button start? Back?
		// drivetrain).deadlineWith(leds.enableState(LED_STATES.CLIMBING)));

		/* BIAS */
		new Trigger(() -> coPilot.getPOV() == 0)
			.onTrue(new InstantCommand(() -> pivot.increaseBias())); // UP
		new Trigger(() -> coPilot.getPOV() == 180)
			.onTrue(new InstantCommand(() -> pivot.decreaseBias())); // DOWN

		new Trigger(() -> coPilot.getPOV() == 90)
			.onTrue(new InstantCommand(() -> flywheel.increaseBias())); // RIGHT
		new Trigger(() -> coPilot.getPOV() == 270)
			.onTrue(new InstantCommand(() -> flywheel.decreaseBias())); // LEFT

		new Trigger(coPilot::getRightBumper)
			.whileTrue(new Index(indexer, () -> IndexerConstants.INDEXER_MANUAL_POWER));
		new Trigger(coPilot::getLeftBumper)
			.whileTrue(new Index(indexer, () -> -IndexerConstants.INDEXER_MANUAL_POWER));

		/* Other */
		new Trigger(
				() -> (limelights.getStopMe().hasTarget() || limelights.getChamps().hasTarget()))
				.whileTrue(leds.enableState(LED_STATES.HAS_VISION));
		new Trigger(() -> collector.hasPiece())
				.whileTrue(leds.enableState(LED_STATES.HAS_PIECE))
				.onTrue(leds.enableState(LED_STATES.COLLECTED).withTimeout(2));

		new Trigger(() -> DriverStation.isAutonomousEnabled()).whileTrue(new CollisionDetection(
				drivetrain, CollisionType.AUTON));

		new Trigger(() -> LightningShuffleboard.getBool("Swerve", "Swap", false))
			.onTrue(new InstantCommand(() -> drivetrain.swap(driver, coPilot)))
			.onFalse(new InstantCommand(() -> drivetrain.swap(driver, coPilot)));
	}

	@Override
	protected void configureDefaultCommands() {
		/* driver */
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand(drivetrain.applyPercentRequestField(
				() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())
				.alongWith(new CollisionDetection(drivetrain, CollisionType.TELEOP)));

		/* copilot */
		collector.setDefaultCommand(
				new Collect(() -> MathUtil.applyDeadband((coPilot.getRightTriggerAxis() - coPilot.getLeftTriggerAxis()), ControllerConstants.DEADBAND), collector, indexer));
		// climber.setDefaultCommand(new ManualClimb(() -> coPilot.getLeftY(),() ->
		// coPilot.getRightY(), climber));
	}

	protected Command getAutonomousCommand() {
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
		SystemTest.registerTest("Drive Test",
				new DrivetrainSystemTest(drivetrain, DrivetrainConstants.SYS_TEST_SPEED_DRIVE));
		SystemTest.registerTest("Azimuth Test",
				new TurnSystemTest(drivetrain, DrivetrainConstants.SYS_TEST_SPEED_TURN));

		// SystemTest.registerTest("Collector Test", new CollectorSystemTest(collector,
		// Constants.CollectorConstants.COLLECTOR_SYSTEST_POWER));

		// TODO make pivot system test

		// SystemTest.registerTest("Flywheel Test", new FlywheelSystemTest(flywheel,
		// collector,
		// indexer, pivot, Constants.FlywheelConstants.SYS_TEST_SPEED));

		// SystemTest.registerTest("Climb Test", new ClimbSystemTest(climber,
		// Constants.ClimbConstants.CLIMB_SYSTEST_POWER));

		// Sing chooser SendableChooser<SystemTestCommand> songChooser = new
		// SendableChooser<>();
		// songChooser.setDefaultOption(MusicConstants.BOH_RHAP_FILEPATH,
		// new SingSystemTest(drivetrain, MusicConstants.BOH_RHAP_FILEPATH, sing));
		// for (String filepath : MusicConstants.SET_LIST) {
		// songChooser.addOption(filepath, new SingSystemTest(drivetrain, filepath,
		// sing));
		// }

		// LightningShuffleboard.set("SystemTest", "Songs List", songChooser);

		// songChooser.onChange((SystemTestCommand command) -> {
		// if (command != null) {
		// command.schedule();
		// }
		// });

		// songChooser.close();
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

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PathFindingConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.Constants.TunerConstants;
import frc.robot.command.AutonSmartCollect;
import frc.robot.command.ChasePieces;
import frc.robot.command.Collect;
import frc.robot.command.CollectAndGo;
import frc.robot.command.ComboPoint;
import frc.robot.command.HasPieceAuto;
import frc.robot.command.Index;
import frc.robot.command.ManualClimb;
import frc.robot.command.PathFindToAuton;
import frc.robot.command.PathToPose;
// import frc.robot.command.ReverseChasePieces;
import frc.robot.command.Sing;
import frc.robot.command.SmartCollect;
import frc.robot.command.UpdateOrientation;
import frc.robot.command.stopDrive;
import frc.robot.command.shoot.AmpShot;
import frc.robot.command.shoot.FlywheelIN;
import frc.robot.command.shoot.NotePass;
import frc.robot.command.shoot.PointBlankShot;
import frc.robot.command.shoot.SmartShoot;
import frc.robot.command.shoot.Stow;
import frc.robot.command.shoot.Tune;
import frc.robot.command.shoot.preAim;
import frc.robot.command.shoot.AutonCand.AmpShotAuton;
import frc.robot.command.shoot.AutonCand.Cand35;
import frc.robot.command.shoot.AutonCand.CandC1;
import frc.robot.command.shoot.AutonCand.CandC2;
import frc.robot.command.shoot.AutonCand.CandC3;
import frc.robot.command.shoot.AutonCand.PointBlankShotAuton;
import frc.robot.command.tests.CollectorSystemTest;
import frc.robot.command.tests.DrivetrainSystemTest;
import frc.robot.command.tests.FlywheelSystemTest;
import frc.robot.command.tests.IndexerSystemTest;
import frc.robot.command.tests.TurnSystemTest;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PivotMercury;
import frc.robot.subsystems.PivotRhapsody;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

	public static XboxControllerFilter driver;
	public static XboxControllerFilter coPilot;
	// public static Joystick buttonBox;

	// Subsystems
	public Swerve drivetrain;
	private Limelights limelights;
	private Collector collector;
	private Flywheel flywheel;
	public Pivot pivot;
	private Indexer indexer;
	private Climber climber;
	LEDs leds;
	Orchestra sing;

	private SendableChooser<Command> autoChooser;
	SwerveRequest.FieldCentric drive;
	SwerveRequest.FieldCentric slow;
	SwerveRequest.RobotCentric driveRobotCentric;
	SwerveRequest.RobotCentric slowRobotCentric;
	SwerveRequest.PointWheelsAt point;
	Telemetry logger;

	private Boolean triggerInit;
	private static double bias = 0d;

	@Override
	protected void initializeSubsystems() {
		driver = new XboxControllerFilter(ControllerConstants.DriverControllerPort,
				Constants.ControllerConstants.DEADBAND, -1, 1,
				XboxControllerFilter.filterMode.SQUARED); // Driver controller
		coPilot = new XboxControllerFilter(ControllerConstants.CopilotControllerPort,
				Constants.ControllerConstants.DEADBAND, -1, 1,
				XboxControllerFilter.filterMode.SQUARED); // CoPilot controller
		// buttonBox = new Joystick(ControllerConstants.ButtonBoxControllerPort);

		drivetrain = TunerConstants.getDrivetrain();
		limelights = new Limelights();
		limelights.setApplyVisionUpdate(drivetrain::applyVisionPose);

		collector = new Collector();
		flywheel = new Flywheel();
		pivot = Constants.IS_MERCURY ? new PivotMercury() : new PivotRhapsody();
		indexer = new Indexer(collector);
		if (!Constants.IS_MERCURY) {
			climber = new Climber();
		}
		leds = new LEDs();
		sing = new Orchestra();

		triggerInit = false;

		point = new SwerveRequest.PointWheelsAt();
		logger = new Telemetry(DrivetrainConstants.MaxSpeed);

		triggerInit = false;

		boolean setPath = SignalLogger.setPath(Constants.HOOT_PATH).isOK();
		SignalLogger.enableAutoLogging(true); // TODO Return during COMPS
		boolean startedLogs = SignalLogger.start().isOK();

		if (startedLogs && setPath) {
			System.out.println("STARTED HOOT LOG");
		} else {
			System.out.println("FAILED TO START HOOT LOG");
		}
	}

	@Override
	protected void initializeNamedCommands() {
		NamedCommands.registerCommand("Disable-Vision",
				new InstantCommand(() -> drivetrain.disableVision()));
		NamedCommands.registerCommand("Enable-Vision",
				new InstantCommand(() -> drivetrain.enableVision()));
		NamedCommands.registerCommand("led-Shoot",
				leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5));

		NamedCommands.registerCommand("Cand-Sub",
				new PointBlankShotAuton(flywheel, pivot, indexer)
						.deadlineWith(leds.enableState(LED_STATES.SHOOTING).withTimeout(1)));
		NamedCommands.registerCommand("Cand-C1", new CandC1(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-C2", new CandC2(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-C3", new CandC3(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Cand-3.5", new Cand35(flywheel, pivot, indexer, collector));
		NamedCommands.registerCommand("AMP", new AmpShotAuton(flywheel, pivot, indexer));
		NamedCommands.registerCommand("Stow", new Stow(flywheel, pivot));
		NamedCommands.registerCommand("Smart-Shoot",
				new SmartShoot(flywheel, pivot, drivetrain, indexer, collector, leds)
						.alongWith(leds.enableState(LED_STATES.SHOOTING).withTimeout(0.5)));
		NamedCommands.registerCommand("preAim", new preAim(flywheel, pivot, drivetrain));
		NamedCommands.registerCommand("Chase-Pieces",
				new ChasePieces(drivetrain, collector, indexer, limelights));
		NamedCommands.registerCommand("Smart-Collect",
				new AutonSmartCollect(() -> 0.5, () -> 0.6, collector, indexer)
						.deadlineWith(leds.enableState(LED_STATES.COLLECTING).withTimeout(1)));
		NamedCommands.registerCommand("Index-Up", new Index(() -> IndexerConstants.INDEXER_DEFAULT_POWER, indexer));
		NamedCommands.registerCommand("PathFind", new PathToPose(PathFindingConstants.TEST_POSE, drivetrain));
		NamedCommands.registerCommand("Collect-And-Go", new CollectAndGo(collector, flywheel, indexer));
		NamedCommands.registerCommand("Point-At-Speaker",
				new ComboPoint(DrivetrainConstants.SPEAKER_POSE, drivetrain, driver, limelights, 0));
		NamedCommands.registerCommand("Point-At-Speaker-left",
				new ComboPoint(DrivetrainConstants.SPEAKER_POSE, drivetrain, driver, limelights, -3));
		NamedCommands.registerCommand("Has-Piece", new HasPieceAuto(indexer));
		NamedCommands.registerCommand("Stop-Drive", new stopDrive(drivetrain));
		NamedCommands.registerCommand("Stop-Flywheel", new FlywheelIN(flywheel));
		NamedCommands.registerCommand("Bias-Down", new InstantCommand(() -> pivot.decreaseBias()));
		NamedCommands.registerCommand("End-Kama", new InstantCommand(() -> flywheel.endKama()));
		NamedCommands.registerCommand("Start-Kama", new InstantCommand(() -> flywheel.startKama()));
		// NamedCommands.registerCommand("Reverse-Cheese-Paste", new ReverseChasePieces(drivetrain, collector, indexer, limelights));

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
				.whileTrue(drivetrain.applyPercentRequestRobot(() -> -driver.getLeftY(),
						() -> -driver.getLeftX(), () -> -driver.getRightX()))
				.onFalse(new InstantCommand(() -> drivetrain.setRobotCentricControl(false)));

		// enables slow mode for driving
		new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
				.onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)))
				.onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

		// sets field relative forward to the direction the robot is facing
		new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain
				.runOnce(drivetrain::seedFieldRelative).andThen(new InstantCommand(() -> drivetrain
						.setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(0))))));

		// makes the robot chase pieces
		new Trigger(driver::getRightBumper).whileTrue(
				new ChasePieces(drivetrain, collector, indexer, limelights)
						.deadlineWith(leds.enableState(LED_STATES.CHASING)));

		// new Trigger(driver::getRightBumper)
		// .whileTrue(new PathFindToAuton(PathPlannerPath.fromPathFile("PathFind-AMP"),
		// drivetrain, driver));

		// parks the robot
		new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.brake()));

		// smart shoot for the robot
		new Trigger(driver::getAButton)
				.whileTrue(new SmartShoot(flywheel, pivot, drivetrain, indexer, collector, leds)
						.deadlineWith(leds.enableState(LED_STATES.SHOOTING)));

		// new Trigger(driver::getBButton).whileTrue(new PathFindToAuton(
		// PathPlannerPath.fromPathFile("PathFind-AMP"), drivetrain));
		new Trigger(driver::getBButton)
				.whileTrue(new PathFindToAuton(PathPlannerPath.fromPathFile("PathFind-AMP"), drivetrain));

		// new Trigger(driver::getYButton) // TODO Make align to TRAP

		// new Trigger(driver::getLeftBumper)
		// .whileTrue(new PointAtPoint(DrivetrainConstants.SPEAKER_POSE, drivetrain,
		// driver));
		new Trigger(driver::getLeftBumper).whileTrue(
				new ComboPoint(DrivetrainConstants.SPEAKER_POSE, drivetrain, driver, limelights, 0d));

		// new Trigger(driver::getYButton)
		// .whileTrue(new MoveToPose(AutonomousConstants.TARGET_POSE, drivetrain));

		new Trigger(() -> driver.getPOV() == 0).toggleOnTrue(leds.enableState(LED_STATES.DISABLED));

		/* COPILOT */
		new Trigger(coPilot::getBButton)
				.whileTrue(new InstantCommand(() -> flywheel.stop(), flywheel)
						.andThen(new SmartCollect(() -> 0.65, () -> 0.9, collector, indexer, pivot, flywheel))
						.deadlineWith(leds.enableState(LED_STATES.COLLECTING)));

		// new Trigger(coPilot::getBButton)
		// 	.whileTrue(new AutonSmartCollect(() -> 0.65, () -> 0.75, collector, indexer)
			// .deadlineWith(leds.enableState(LED_STATES.COLLECTING).withTimeout(1)));

		// cand shots for the robot
		new Trigger(coPilot::getXButton)
				.whileTrue(new PointBlankShot(flywheel, pivot).deadlineWith(leds.enableState(LED_STATES.SHOOTING)));
		// new Trigger(coPilot::getYButton).whileTrue(new PivotUP(pivot));
		new Trigger(coPilot::getYButton).whileTrue(new NotePass(flywheel, pivot)
				.deadlineWith(leds.enableState(LED_STATES.SHOOTING)));
		// new Trigger(coPilot::getYButton).whileTrue(new Tune(flywheel, pivot)
		// .deadlineWith(leds.enableState(LED_STATES.SHOOTING)));
		new Trigger(coPilot::getAButton).whileTrue(new AmpShot(flywheel, pivot)
				.deadlineWith(leds.enableState(LED_STATES.SHOOTING)));
		new Trigger(coPilot::getRightBumper)
				.whileTrue(new Index(() -> IndexerConstants.INDEXER_DEFAULT_POWER, indexer));
		new Trigger(coPilot::getLeftBumper)
				.whileTrue(new Index(() -> -IndexerConstants.INDEXER_DEFAULT_POWER, indexer)
						.deadlineWith(new FlywheelIN(flywheel)));

		/* BIAS */
		new Trigger(() -> coPilot.getPOV() == 0)
				.onTrue(new InstantCommand(() -> pivot.increaseBias())); // UP
		new Trigger(() -> coPilot.getPOV() == 180)
				.onTrue(new InstantCommand(() -> pivot.decreaseBias())); // DOWN

		new Trigger(() -> coPilot.getPOV() == 90)
				.onTrue(new InstantCommand(() -> flywheel.increaseBias())); // RIGHT
		new Trigger(() -> coPilot.getPOV() == 270)
				.onTrue(new InstantCommand(() -> flywheel.decreaseBias())); // LEFT

		new Trigger(() -> coPilot.getBackButton() && coPilot.getStartButton())
				.onTrue(new InstantCommand(() -> pivot.resetBias())
						.alongWith(new InstantCommand(() -> flywheel.resetBias())));

		/* Other */
		new Trigger(
				() -> ((limelights.getStopMe().hasTarget() || limelights.getChamps().hasTarget())
						&& DriverStation.isEnabled()))
				.whileTrue(leds.enableState(LED_STATES.HAS_VISION));
		new Trigger(() -> indexer.getEntryBeamBreakState() || indexer.getExitBeamBreakState()
				|| collector.getEntryBeamBreakState())
				.whileTrue(leds.enableState(LED_STATES.HAS_PIECE))
				.whileTrue(leds.enableState(LED_STATES.COLLECTED).withTimeout(2));
		new Trigger(() -> drivetrain.isInField() && triggerInit)
				.whileFalse(leds.enableState(LED_STATES.BAD_POSE));
		new Trigger(() -> !drivetrain.isStable() && DriverStation.isDisabled()
				&& !(limelights.getStopMe().getBlueAlliancePose().getMoreThanOneTarget()
						|| limelights.getChamps().getBlueAlliancePose().getMoreThanOneTarget()))
				.whileTrue(leds.enableState(LED_STATES.BAD_POSE));
		new Trigger(() -> DriverStation.isDisabled()
				&& !(limelights.getStopMe().getBlueAlliancePose().getMoreThanOneTarget()
						|| limelights.getChamps().getBlueAlliancePose().getMoreThanOneTarget()))
				.whileTrue(leds.enableState(LED_STATES.BAD_POSE));
		new Trigger(() -> !drivetrain.isStable() && DriverStation.isDisabled()
				&& (limelights.getStopMe().getBlueAlliancePose().getMoreThanOneTarget()
						|| limelights.getChamps().getBlueAlliancePose().getMoreThanOneTarget()))
				.whileTrue(leds.enableState(LED_STATES.GOOD_POSE));
		triggerInit = true;

		new Trigger(() -> collector.getEntryBeamBreakState())
				.whileTrue(leds.enableState(LED_STATES.COLLECTOR_BEAMBREAK));
		new Trigger(() -> indexer.getEntryBeamBreakState())
				.whileTrue(leds.enableState(LED_STATES.INDEXER_ENTER_BEAMBREAK));
		new Trigger(() -> indexer.getExitBeamBreakState())
				.whileTrue(leds.enableState(LED_STATES.INDEXER_EXIT_BEAMBREAK));
		new Trigger(() -> pivot.getForwardLimit())
				.whileTrue(leds.enableState(LED_STATES.PIVOT_BOTTOM_SWITCH));
		new Trigger(() -> pivot.getReverseLimit())
				.whileTrue(leds.enableState(LED_STATES.PIVOT_TOP_SWITCH));

		// new Trigger(() -> DriverStation.isAutonomousEnabled())
		// .whileTrue(new CollisionDetection(drivetrain, CollisionType.AUTON));

		// new Trigger(() -> LightningShuffleboard.getBool("Swerve", "Swap", false))
		// .onTrue(new InstantCommand(() -> drivetrain.swap(driver, coPilot)))
		// .onFalse(new InstantCommand(() -> drivetrain.swap(driver, coPilot)));
		// BLUE Alliance set
		// new Trigger(() -> LightningShuffleboard.getBool("Auton", "POSE RED A",
		// false))
		// .onTrue(new InstantCommand(
		// () ->
		// drivetrain.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_A_STARTPOSE_RED)));
		// new Trigger(() -> LightningShuffleboard.getBool("Auton", "POSE RED B",
		// false))
		// .onTrue(new InstantCommand(
		// () ->
		// drivetrain.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_B_STARTPOSE_RED)));
		// new Trigger(() -> LightningShuffleboard.getBool("Auton", "POSE RED C",
		// false))
		// .onTrue(new InstantCommand(
		// () ->
		// drivetrain.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_C_STARTPOSE_RED)));

		/* Button Box */
		// new Trigger(() -> buttonBox.getRawButton(ButtonBox.PINK)).whileTrue(new
		// PivotUP(pivot));
		// new Trigger(() -> buttonBox.getRawAxis(ButtonBox.GRAY_BOTTOMLEFT) ==
		// 1).whileTrue(new PivotUP(pivot));

		// BLUE Alliance set
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE BLUE A", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_A_STARTPOSE_BLUE)));
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE BLUE B", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_B_STARTPOSE_BLUE)));
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE BLUE C", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_C_STARTPOSE_BLUE)));

		// BLUE Alliance set
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE RED A", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_A_STARTPOSE_RED)));
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE RED B", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_B_STARTPOSE_RED)));
		new Trigger(
				() -> DriverStation.isDisabled() ? LightningShuffleboard.getBool("Auton", "POSE RED C", false) : false)
				.onTrue(new InstantCommand(() -> drivetrain
						.setDrivetrainPose(AutonomousConstants.SOURCE_SUB_C_STARTPOSE_RED)));
	}

	@Override
	protected void configureDefaultCommands() {
		/* driver */
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand(drivetrain.applyPercentRequestField(() -> -driver.getLeftY(),
				() -> -driver.getLeftX(), () -> -driver.getRightX()));
		// .alongWith(new CollisionDetection(drivetrain, CollisionType.TELEOP)));

		/* copilot */
		collector.setDefaultCommand(new Collect(() -> MathUtil.applyDeadband(
				(coPilot.getRightTriggerAxis() - coPilot.getLeftTriggerAxis()),
				ControllerConstants.DEADBAND), collector));

		// climber.setDefaultCommand(
		// new SmartClimb(climber, drivetrain, pivot, leds, () -> -coPilot.getLeftY(),
		// () -> -coPilot.getRightY(),
		// coPilot::getYButton).deadlineWith(leds.enableState(LED_STATES.CLIMBING)));

		if (!Constants.IS_MERCURY) {
			climber.setDefaultCommand(new ManualClimb(() -> -coPilot.getRightY(), () -> -coPilot.getLeftY(), climber));
		}

		// limelights.setDefaultCommand(new UpdateOrientation(limelights, drivetrain));
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
				new DrivetrainSystemTest(drivetrain, DrivetrainConstants.SYS_TEST_SPEED_DRIVE)); // to
																									// be
																									// tested
		SystemTest.registerTest("Azimuth Test",
				new TurnSystemTest(drivetrain, DrivetrainConstants.SYS_TEST_SPEED_TURN));

		// SystemTest.registerTest("Single Note Cycle", new CycleSytemTest(collector,
		// indexer, pivot, flywheel, () -> 0.5d, () -> 0.6d, () -> 250));

		SystemTest.registerTest("Collector Test", new CollectorSystemTest(collector,
				Constants.CollectorConstants.COLLECTOR_SYSTEST_POWER));

		// SystemTest.registerTest("Pivot 90 Degrees", new PivotAngleTest(pivot,
		// Constants.PivotConstants.PIVOT_SYSTEST_ANGLE));

		SystemTest.registerTest("Flywheel Test", new FlywheelSystemTest(flywheel, collector,
				indexer, pivot, Constants.FlywheelConstants.FLYWHEEL_SYSTEST_RPM));

		SystemTest.registerTest("Indexer Test",
				new IndexerSystemTest(indexer, Constants.IndexerConstants.INDEXER_SYSTEST_POWER));

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
		if (!DriverStation.isAutonomous()) {
			return new StartEndCommand(() -> {
				driver.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
				driver.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
			}, () -> {
				driver.setRumble(GenericHID.RumbleType.kRightRumble, 0);
				driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
			});
		} else {
			return new InstantCommand();
		}
	}

	public static Command hapticCopilotCommand() {
		if (!DriverStation.isAutonomous()) {
			return new StartEndCommand(() -> {
				coPilot.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
				coPilot.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
			}, () -> {
				coPilot.setRumble(GenericHID.RumbleType.kRightRumble, 0);
				coPilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
			});
		} else {
			return new InstantCommand();
		}
	}
}

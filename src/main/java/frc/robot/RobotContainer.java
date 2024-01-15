package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.Constants.drivetrainConstants;

import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class RobotContainer extends LightningContainer {
	private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    /* Setting up bindings for necessary control of the swerve drive platform */
    XboxController driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
    XboxController coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller

	Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
	Collector collector = new Collector();
	// Flywheel flywheel = new Flywheel();
	// Pivot pivot = new Pivot();
	// Shooter shooter = new Shooter(pivot, flywheel);

	// TODO I want field-centric driving in open loop WE NEED TO FIGURE OUT WHAT Change beacuse with open loop is gone
	SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); 
	SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	Telemetry logger = new Telemetry(drivetrainConstants.MaxSpeed);

	

	@Override
	protected void configureButtonBindings() {
		NamedCommands.registerCommand("CollectIN", new Collect(collector, () -> 1d));

		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);


		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(driver::getXButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // TODO create function to reset Heading
	}

        @Override
        protected Command configureAutonomousCommands() {
			return autoChooser.getSelected();
		}
	@Override
	protected void configureDefaultCommands() {
		drivetrain.registerTelemetry(logger::telemeterize);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), 0.1)
								* drivetrainConstants.MaxSpeed) // Drive forward with negative Y (forward)
						.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1)
								* drivetrainConstants.MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1)
								* drivetrainConstants.MaxAngularRate))); // Drive counterclockwise with negative X (left)

		collector.setDefaultCommand(new Collect(() -> (coPilot.getRightTriggerAxis() - coPilot.getLeftTriggerAxis()), collector));
	}

	@Override
	protected void configureAutonomousCommands() {}

	@Override
	protected void releaseDefaultCommands() {}

	@Override
	protected void initializeDashboardCommands() {}

	@Override
	protected void configureFaultCodes() {}

	@Override
	protected void configureFaultMonitors() {}

	@Override
	protected void configureSystemTests() {}
}

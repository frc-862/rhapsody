package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.*;
import frc.robot.subsystems.Collector;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.drivetrainConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class RobotContainer extends LightningContainer {
	private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    /* Setting up bindings for necessary control of the swerve drive platform */
    XboxController driver = new XboxController(ControllerConstants.DriverControllerPort); // Driver controller
    XboxController coPilot = new XboxController(ControllerConstants.CopilotControllerPort); // CoPilot controller

    Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain

	SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); //TODO I want field-centric driving in open loop     WE NEED TO FIGURE OUT WHAT Change beacuse with open loop is gone
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(drivetrainConstants.MaxSpeed);
    Collector collector = new Collector();

	
	

	@Override
	protected void configureButtonBindings() {
		NamedCommands.registerCommand("CollectIN", new Collect(collector, () -> 1d));

		LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);


		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(driver::getXButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // TODO create function to reset Heading

        // Run collector in/out
        new Trigger(driver::getLeftBumper).whileTrue(new Collect(collector, () -> -1d));
        new Trigger(driver::getRightBumper).whileTrue(new Collect(collector, () -> 1d));
    }
	
        @Override
        protected void configureDefaultCommands() {
            drivetrain.registerTelemetry(logger::telemeterize);

            drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(
                -MathUtil.applyDeadband(driver.getLeftY(), 0.1) * drivetrainConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * drivetrainConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * drivetrainConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));
	}

        @Override
        protected Command configureAutonomousCommands() {
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
        protected void configureSystemTests() {}
}

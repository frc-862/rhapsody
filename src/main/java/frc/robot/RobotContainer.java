package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.*;
import frc.robot.subsystems.Collector;
import frc.thunder.LightningContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.drivetrainConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
        
    public class RobotContainer extends LightningContainer {
        /* Setting up bindings for necessary control of the swerve drive platform */
        XboxController driver = new XboxController(ControllerConstants.DriverControllerPort); // My joystick

        Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
        Collector collector = new Collector();
	
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); //TODO I want field-centric driving in open loop   WE NEED TO FIGURE OUT WHAT Change beacuse with open loop is gone
  	    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  	    Telemetry logger = new Telemetry(drivetrainConstants.MaxSpeed);

	
	@Override
	protected void configureButtonBindings() {	  
		new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		new Trigger(driver::getXButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyro())); // TODO create function to reset Heading
        new Trigger(driver::getRightBumper).whileTrue(new Collect(collector));
		drivetrain.registerTelemetry(logger::telemeterize);
    }
	

    
    protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
		  drivetrain.applyRequest(() -> drive.withVelocityX(
			-MathUtil.applyDeadband(driver.getLeftY(), 0.1) * drivetrainConstants.MaxSpeed) // Drive forward with negative Y (forward)
			  .withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * drivetrainConstants.MaxSpeed) // Drive left with negative X (left)
			  .withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * drivetrainConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
		  ));

        // Get collector entry beam break state, then run collector if object is present
        collector.setDefaultCommand(new RunCommand(() -> {
            if (!collector.getEntryBeamBreakState()) {
                collector.setPower(1d);
            } else {
                collector.stop();
            }
        }, collector)); // TODO teach not to do this
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

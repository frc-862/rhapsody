package frc.robot;

import java.util.ResourceBundle.Control;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.command.NoteCollect;
import frc.robot.command.Shoot;
import frc.robot.command.TagAutoAlign;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.PivotRhapsody;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.vision.Limelight;

public class RobotContainer extends LightningContainer{
    //init subsystems
    public Climber climber;
    public Collector collector; 
    public Flywheel flywheel;
    public Indexer indexer;
    public PivotRhapsody pivot;
    public Limelights limelights;
    public XboxController driver;
    public XboxController copilot;
    public Swerve drivetrain;
    public LEDs leds;

    @Override
    protected void initializeSubsystems() {
        collector = new Collector();
        indexer = new Indexer();
        flywheel = new Flywheel();
        drivetrain = TunerConstants.getDrivetrain();
        pivot = new PivotRhapsody();
        leds = new LEDs();
        limelights = new Limelights();

        driver = new XboxController(Constants.ControllerConstants.DriverControllerPort);
        copilot = new XboxController(Constants.ControllerConstants.CopilotControllerPort);

    }
    @Override
    protected void initializeNamedCommands() {

    }
    @Override
    protected void configureButtonBindings() {
        new Trigger(driver::getBButton).onTrue(new InstantCommand(() -> pivot.setTargetAngle(pivot.getStowAngle()), pivot));

        new Trigger(() -> driver.getStartButtonPressed() && driver.getBackButtonPressed()).onTrue(
            new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(drivetrain.getRotation3d().toRotation2d()), drivetrain)
        );

        new Trigger(() -> driver.getYButton()).onTrue(new InstantCommand(() -> pivot.setTargetAngle(60), pivot));

        new Trigger(() -> driver.getPOV() == 180).onTrue(new InstantCommand(() -> flywheel.stopCoast(), flywheel));
        new Trigger(() -> driver.getPOV() == 270).onTrue(new InstantCommand(() -> flywheel.startCoast(), flywheel));

        new Trigger(() -> driver.getAButton()).onTrue(new Shoot(flywheel, indexer).raceWith(leds.enableState(LED_STATES.SHOOTING)));
        new Trigger(() -> driver.getXButton()).onTrue(new NoteCollect(collector, indexer).andThen(leds.enableState(LED_STATES.COLLECTED).withTimeout(1)));

        

        new Trigger(() -> driver.getLeftBumper()).onTrue(new TagAutoAlign(limelights, pivot, drivetrain, () -> driver.getLeftX(), () -> driver.getLeftY()).andThen(leds.enableState(LED_STATES.GOOD_POSE).withTimeout(1)));
        // new Trigger(() -> driver.getLeftBumper()).onTrue((new Shoot(flywheel, indexer)
        // .deadlineWith(new TagAutoAlign(limelights, pivot, drivetrain, () -> driver.getLeftX(), () -> driver.getLeftY()))));
    }
    @Override
    protected void configureSystemTests() {
        // TODO Auto-generated method stub
    }
    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyPercentRequestField(
            () -> MathUtil.applyDeadband(-driver.getLeftY() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND), 
            () -> MathUtil.applyDeadband(-driver.getLeftX() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND), 
            () -> MathUtil.applyDeadband(-driver.getRightX() * drivetrain.getRotMult(), ControllerConstants.DEADBAND)));

        // indexer.setDefaultCommand(new SpinIndex(indexer, () -> driver.getLeftTriggerAxis() * 0.5));
        // flywheel.setDefaultCommand(new Shoot(flywheel, () -> driver.getRightTriggerAxis()));
    }
    @Override
    protected void releaseDefaultCommands() {
        // TODO Auto-generated method stub
    }
    @Override
    protected void initializeDashboardCommands() {
        // TODO Auto-generated method stub
    }
    @Override
    protected void configureFaultCodes() {
        // TODO Auto-generated method stub
    }
    @Override
    protected void configureFaultMonitors() {
        // TODO Auto-generated method stub
    }
    @Override
    protected Command getAutonomousCommand() {
        // TODO Auto-generated method stub
        return new NoteCollect(collector, indexer);
    }
}

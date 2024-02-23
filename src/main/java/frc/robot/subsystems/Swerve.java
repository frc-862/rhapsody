package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    private final Limelights limelightSubsystem;

    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean slowMode = false;
    private boolean disableVision = false;
    private boolean robotCentricControl = false;
    private double maxSpeed = DrivetrainConstants.MaxSpeed;
    private double maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Limelights limelightSubsystem, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        this.limelightSubsystem = limelightSubsystem;

        initLogging();

        configurePathPlanner();
    }

    /* DRIVE METHODS */

    /**
     * Apply a percentage Field centric request to the drivetrain
     * @param x the x, percent of max velocity (-1,1)
     * @param y the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return run(() -> this.setControl(driveField
                .withVelocityX(x.getAsDouble() * maxSpeed)
                .withVelocityY(y.getAsDouble() * maxSpeed)
                .withRotationalRate(rot.getAsDouble() * maxAngularRate)));
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic
     * @param x the x velocity m/s
     * @param y the y velocity m/s
     * @param rot the rotational velocity in rad/s
     */
    public void setField(double x, double y, double rot) {
        this.setControl(driveField
            .withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(rot));
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic,
     * Allows driving normally and pid control of rotation
     * @param x the x, percent of max velocity (-1,1)
     * @param y the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity rad/s
     */
    public void setFieldDriver(double x, double y, double rot) {
        this.setControl(driveField
            .withVelocityX(x * maxSpeed)
            .withVelocityY(y * maxSpeed)
            .withRotationalRate(rot));
    }

    /**
     * Apply a percentage Robot centric request to the drivetrain
     * @param x the x, percent of max velocity (-1,1)
     * @param y the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity (-1,1)
     * @return the request to drive for the drivetrain
     */
    public Command applyPercentRequestRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return run(() -> this.setControl(driveRobot
                .withVelocityX(x.getAsDouble() * maxSpeed)
                .withVelocityY(y.getAsDouble() * maxSpeed)
                .withRotationalRate(rot.getAsDouble() * maxAngularRate)));
    }

    /**
     * Apply a Robot centric request to the drivetrain run in periodic
     * @param x the x velocity m/s
     * @param y the y velocity m/s
     * @param rot the rotational velocity in rad/s
     */
    public void setRobot(double x, double y, double rot) {
        this.setControl(driveRobot
            .withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(rot));
    }

    /**
     * Sets the robot in park mode
     */
    public void brake() {
        this.setControl(brake);
    }

    /**
     * Apply a request to the drivetrain
     * @param requestSupplier the SwerveRequest to apply
     * @return the request to drive for the drivetrain
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume */
        updateSimState(0.01, 12);
    }

    @Override
    public void periodic() {
        // TODO Remove the unecessary shuffleboard stuff eventually
        if (!disableVision) {
            var pose = limelightSubsystem.getPoseQueue().poll();
            while (pose != null) {
                // High confidence => 0.3 
                // Low confidence => 18 
                // theta trust IMU, use 500 degrees

                double confidence = 18.0;
                if (pose.getMoreThanOneTarget() && pose.getDistance() < 3) {
                    confidence = 0.3;
                } else if (pose.getMoreThanOneTarget()) {
                    confidence = 0.3 + ((pose.getDistance() - 3)/5 * 18);
                } else if (pose.getDistance() < 2) {
                    confidence = 1.0 + (pose.getDistance()/2 * 5.0);
                }
                
                addVisionMeasurement(pose.toPose2d(), pose.getFPGATimestamp(), VecBuilder.fill(confidence, confidence, Math.toRadians(500)));
                pose = limelightSubsystem.getPoseQueue().poll();

                // TODO remove once test new vision checks on Monday
                LightningShuffleboard.setDouble("Swerve", "PoseX", pose.toPose2d().getX());
                LightningShuffleboard.setDouble("Swerve", "PoseY", pose.toPose2d().getY());
                LightningShuffleboard.setDouble("Swerve", "PoseTime",  pose.getFPGATimestamp());
                LightningShuffleboard.setDouble("Swerve", "distance",  pose.getDistance());
                LightningShuffleboard.setBool("Swerve", "MultipleTargets",  pose.getMoreThanOneTarget());
            }
        }
    }

    private void initLogging() {
        // TODO Remove the unecessary shuffleboard stuff eventually
        LightningShuffleboard.setDoubleSupplier("Swerve", "Timer", () -> Timer.getFPGATimestamp());
        LightningShuffleboard.setDoubleSupplier("Swerve", "Robot Heading", () -> getPigeon2().getAngle());
        LightningShuffleboard.setDoubleSupplier("Swerve", "Odo X", () -> getState().Pose.getX());
        LightningShuffleboard.setDoubleSupplier("Swerve", "Odo Y", () -> getState().Pose.getY());

        LightningShuffleboard.setBoolSupplier("Swerve", "Slow mode", () -> slowMode);
        LightningShuffleboard.setBoolSupplier("Swerve", "Robot Centric", () -> isRobotCentricControl());

        LightningShuffleboard.setBoolSupplier("Sweve", "Tipped", () -> isTipped());

        LightningShuffleboard.setDoubleSupplier("Swerve", "velocity x",
                () -> getPigeon2().getAngularVelocityXDevice().getValueAsDouble());
        LightningShuffleboard.setDoubleSupplier("Swerve", "velocity y",
                () -> getPigeon2().getAngularVelocityYDevice().getValueAsDouble());
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(AutonomousConstants.TRANSLATION_PID,
                        AutonomousConstants.ROTATION_PID, AutonomousConstants.MAX_MODULE_VELOCITY,
                        AutonomousConstants.DRIVE_BASE_RADIUS, new ReplanningConfig(),
                        AutonomousConstants.CONTROL_LOOP_PERIOD),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this); // Subsystem for requirements
    }

    public Supplier<Pose2d> getPose() {
        return () -> getState().Pose;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * @return whether the robot is tipped
     */
    public boolean isTipped() {
        return (Math.abs(
                getPigeon2().getPitch().getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE
                || Math.abs(getPigeon2().getRoll()
                        .getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE);
    }

    /**
     * gets if slow mode is enabled
     * @return if the robot is driving in slow mode
     */
    public boolean inSlowMode() {
        return slowMode;
    }

    /**
     * Set slow mode t/f
     * @param slow boolean if we are in slow mode
     */
    public void setSlowMode(boolean slow) {
        if (slow) {
            maxSpeed = DrivetrainConstants.MaxSpeed * DrivetrainConstants.SLOW_SPEED_MULT;
            maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT
                    * DrivetrainConstants.SLOW_ROT_MULT;
        } else {
            maxSpeed = DrivetrainConstants.MaxSpeed;
            maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT;
        }
    }

    /**
     * Logs if the robot is in robot centric control
     * @param robotCentricControl boolean if the robot is in robot centric control
     */
    public void setRobotCentricControl(boolean robotCentricControl) {
        this.robotCentricControl = robotCentricControl;
    }

    /**
     * Returns if the robot is in robot centric control
     * @return boolean if the robot is in robot centric control
     */
    public boolean isRobotCentricControl() {
        return robotCentricControl;
    }

    /**
     * Swaps the driver and copilot controllers
     * @param driverC the driver controller
     * @param copilotC the copilot controller
     */
    public void swap(XboxControllerFilter driverC, XboxControllerFilter copilotC) {
        XboxControllerFilter temp = driverC;
        RobotContainer.driver = copilotC;
        RobotContainer.coPilot = temp;
    }

    /**
     * Returns if the robot Pose is in Wing
     * @return boolean if the robot is in the wing to start aiming STATE priming
     */
    public boolean inWing() {
        return (getPose().get().getX() < ShooterConstants.FAR_WING_X);
    }

    public void disableVision() {
        disableVision = true;
        System.out.println("Vision Disabled");
    }

    public void enableVision() {
        disableVision = false;
        System.out.println("Vision Enabled");
    }
}

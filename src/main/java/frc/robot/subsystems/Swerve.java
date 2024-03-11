package frc.robot.subsystems;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.CollisionConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.util.Pose4d;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Pair;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean slowMode = false;
    private boolean disableVision = false;
    private boolean robotCentricControl = false;
    private double maxSpeed = DrivetrainConstants.MaxSpeed;
    private double maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT;
    private Translation2d speakerPose = VisionConstants.BLUE_SPEAKER_LOCATION.toTranslation2d();

    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        initLogging();

        configurePathPlanner();

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            speakerPose = VisionConstants.RED_SPEAKER_LOCATION.toTranslation2d();
        }

        // setRampRate();
    }

    private void setRampRate() {
        var config = new TalonFXConfiguration();
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.1;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        for (int i = 0; i < 4; ++i) {
            var module = getModule(i);
            var drive = module.getDriveMotor();
            var steer = module.getSteerMotor();

            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int j = 0; j < 5; ++j) {
                status = drive.getConfigurator().apply(config);
                if (status.isOK()) {
                    break;
                }
            }
            for (int j = 0; j < 5; ++j) {
                status = steer.getConfigurator().apply(config);
                if (status.isOK()) {
                    break;
                }
            }
        }
    }

    public void applyVisionPose(Pose4d pose) {
        if (!disableVision) {
            addVisionMeasurement(pose.toPose2d(), pose.getFPGATimestamp(), pose.getStdDevs());
        }
    }

    /* DRIVE METHODS */

    /**
     * Apply a percentage Field centric request to the drivetrain
     * 
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
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
     * 
     * @param x   the x velocity m/s
     * @param y   the y velocity m/s
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
     * 
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
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
     * 
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
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
     * 
     * @param x   the x velocity m/s
     * @param y   the y velocity m/s
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
     * 
     * @param requestSupplier the SwerveRequest to apply
     * @return the request to drive for the drivetrain
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        // TODO: don't use
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume */
        updateSimState(0.01, 12);
    }

    @SuppressWarnings({"unchecked", "resource"})
    private void initLogging() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Swerve", DrivetrainConstants.DRIVETRAIN_LOG_PERIOD,
                new Pair<String, Object>("Timer", (DoubleSupplier) Timer::getFPGATimestamp),
                new Pair<String, Object>("Robot Heading", (DoubleSupplier) getPigeon2()::getAngle),
                new Pair<String, Object>("Odo X", (DoubleSupplier) getPose()::getX),
                new Pair<String, Object>("Odo Y", (DoubleSupplier) getPose()::getY),
                new Pair<String, Object>("Slow mode", (BooleanSupplier) this::inSlowMode),
                new Pair<String, Object>("Robot Centric", (BooleanSupplier) this::isRobotCentricControl),
                new Pair<String, Object>("Tipped", (BooleanSupplier) this::isTipped),
                new Pair<String, Object>("velocity x", (DoubleSupplier) getPigeon2().getAngularVelocityXDevice()::getValueAsDouble),
                new Pair<String, Object>("velocity y", (DoubleSupplier) getPigeon2().getAngularVelocityYDevice()::getValueAsDouble),
                new Pair<String, Object>("Distance to Speaker", (DoubleSupplier) this::distanceToSpeaker));
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
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

    public Pose2d getPose() {
        var state = getState();
        if (state == null || getState().Pose == null) {
            return new Pose2d();
        }
        return state.Pose;
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
                getPigeon2().getPitch().getValueAsDouble()) > CollisionConstants.TIP_DEADZONE
                || Math.abs(getPigeon2().getRoll()
                        .getValueAsDouble()) > CollisionConstants.TIP_DEADZONE);
    }

    /**
     * gets if slow mode is enabled
     * 
     * @return if the robot is driving in slow mode
     */
    public boolean inSlowMode() {
        return slowMode;
    }

    /**
     * Set slow mode t/f
     * 
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
     * 
     * @param robotCentricControl boolean if the robot is in robot centric control
     */
    public void setRobotCentricControl(boolean robotCentricControl) {
        this.robotCentricControl = robotCentricControl;
    }

    /**
     * Returns if the robot is in robot centric control
     * 
     * @return boolean if the robot is in robot centric control
     */
    public boolean isRobotCentricControl() {
        return robotCentricControl;
    }

    /**
     * Swaps the driver and copilot controllers
     * 
     * @param driverC  the driver controller
     * @param copilotC the copilot controller
     */
    public void swap(XboxControllerFilter driverC, XboxControllerFilter copilotC) {
        XboxControllerFilter temp = driverC;
        RobotContainer.driver = copilotC;
        RobotContainer.coPilot = temp;
    }

    /**
     * Returns if the robot Pose is in Wing
     * 
     * @return boolean if the robot is in the wing to start aiming STATE priming
     */
    public boolean inWing() {
        return (getPose().getX() < ShooterConstants.FAR_WING_X);
    }

    public void disableVision() {
        disableVision = true;
        System.out.println("Vision Disabled");
    }

    public void enableVision() {
        disableVision = false;
        System.out.println("Vision Enabled");
    }

    public double distanceToSpeaker() {
        return speakerPose.getDistance(getPose().getTranslation());
    }
}

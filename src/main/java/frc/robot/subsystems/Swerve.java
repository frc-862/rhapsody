package frc.robot.subsystems;

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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.CollisionConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    static RectangularRegionConstraint field = new RectangularRegionConstraint(
            new Translation2d(0, 0), VisionConstants.FIELD_LIMIT, null);

    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean slowMode = false;
    private boolean disableVision = false;
    private boolean robotCentricControl = false;
    private double maxSpeed = DrivetrainConstants.MaxSpeed;
    private double maxAngularRate = DrivetrainConstants.MaxAngularRate * DrivetrainConstants.ROT_MULT;
    private LinearFilter xFilter = LinearFilter.singlePoleIIR(2, 0.01);
    private LinearFilter yFilter = LinearFilter.singlePoleIIR(2, 0.01);
    private LinearFilter rotFilter = LinearFilter.singlePoleIIR(2, 0.01);
    private Translation2d speakerPose = VisionConstants.BLUE_SPEAKER_LOCATION.toTranslation2d();

    private DoubleLogEntry timerLog;
    private DoubleLogEntry robotHeadingLog;
    private DoubleLogEntry odoXLog;
    private DoubleLogEntry odoYLog;
    private BooleanLogEntry slowModeLog;
    private BooleanLogEntry robotCentricLog;
    private BooleanLogEntry tippedLog;
    private DoubleLogEntry velocityXLog;
    private DoubleLogEntry velocityYLog;
    private DoubleLogEntry distanceToSpeakerLog;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        configurePathPlanner();

        setRampRate();

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        timerLog = new DoubleLogEntry(log, "/Swerve/Timer");
        robotHeadingLog = new DoubleLogEntry(log, "/Swerve/Robot Heading");
        odoXLog = new DoubleLogEntry(log, "/Swerve/Odo X");
        odoYLog = new DoubleLogEntry(log, "/Swerve/Odo Y");
        slowModeLog = new BooleanLogEntry(log, "/Swerve/Slow mode");
        robotCentricLog = new BooleanLogEntry(log, "/Swerve/Robot Centric");
        tippedLog = new BooleanLogEntry(log, "/Swerve/Tipped");
        velocityXLog = new DoubleLogEntry(log, "/Swerve/velocity x");
        velocityYLog = new DoubleLogEntry(log, "/Swerve/velocity y");
        distanceToSpeakerLog = new DoubleLogEntry(log, "/Swerve/Distance to Speaker");

        LightningShuffleboard.setBoolSupplier("Swerve", "Slow Mode", () -> inSlowMode());
        LightningShuffleboard.setBoolSupplier("Swerve", "Robot Centric", () -> isRobotCentricControl());
        LightningShuffleboard.setBoolSupplier("Swerve", "Tipped", () -> isTipped());

        LightningShuffleboard.setDoubleSupplier("Swerve", "Odometry X", () -> getPose().getX());
        LightningShuffleboard.setDoubleSupplier("Swerve", "Odometry Y", () -> getPose().getY());

        LightningShuffleboard.setDoubleSupplier("Swerve", "Robot Heading", () -> getPose().getRotation().getDegrees());
        LightningShuffleboard.setDoubleSupplier("Swerve", "Distance to speaker", () -> distanceToSpeaker());
    }

    private void setRampRate() {
        var config = new TalonFXConfiguration();
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.2;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 50;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -50;

        for (int i = 0; i < 4; ++i) {
            var module = getModule(i);
            var drive = module.getDriveMotor();
            var steer = module.getSteerMotor();

            StatusCode status = StatusCode.StatusCodeNotInitialized;
            StatusCode status1 = StatusCode.StatusCodeNotInitialized;
            for (int j = 0; j < 5; ++j) {
                // Theory is like, it'll refresh, and then apply.
                status1 = drive.getConfigurator().refresh(config);
                // kyle said try refresh, but im pretty sure it's for reading only.
                status = drive.getConfigurator().apply(config);
                if (status.isOK() && status1.isOK()) {
                    break;
                }
            }
            for (int j = 0; j < 5; ++j) {
                status1 = steer.getConfigurator().refresh(config);
                status = steer.getConfigurator().apply(config);
                if (status.isOK() && status1.isOK()) {
                    break;
                }
            }
        }
    }

    @Override
    public void periodic() {
        xFilter.calculate(getPose().getX());
        yFilter.calculate(getPose().getY());
        rotFilter.calculate(getPose().getRotation().getDegrees());

        updateLogging();
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
    public Command applyPercentRequestField(DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier rot) {
        return run(() -> this.setControl(driveField.withVelocityX(x.getAsDouble() * maxSpeed)
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
        this.setControl(driveField.withVelocityX(x).withVelocityY(y).withRotationalRate(rot));
    }

    /**
     * Apply a Field centric request to the drivetrain run in periodic, Allows
     * driving normally and
     * pid control of rotation
     *
     * @param x   the x, percent of max velocity (-1,1)
     * @param y   the y, percent of max velocity (-1,1)
     * @param rot the rotational, percent of max velocity rad/s
     */
    public void setFieldDriver(double x, double y, double rot) {
        this.setControl(driveField.withVelocityX(x * maxSpeed).withVelocityY(y * maxSpeed)
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
    public Command applyPercentRequestRobot(DoubleSupplier x, DoubleSupplier y,
            DoubleSupplier rot) {
        return run(() -> this.setControl(driveRobot.withVelocityX(x.getAsDouble() * maxSpeed)
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
        this.setControl(driveRobot.withVelocityX(x).withVelocityY(y).withRotationalRate(rot));
    }

    /**
     * Sets the robot in park mode
     */
    public void brake() {
        this.setControl(brake);
    }

    public void stop() {
        applyPercentRequestField(() -> 0d, () -> 0d, () -> 0d);
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

    /**
     * update logging
     */
    public void updateLogging() {
        timerLog.append(Timer.getFPGATimestamp());
        robotHeadingLog.append(getPigeon2().getAngle());
        odoXLog.append(getPose().getX());
        odoYLog.append(getPose().getY());
        slowModeLog.append(inSlowMode());
        robotCentricLog.append(isRobotCentricControl());
        tippedLog.append(isTipped());
        velocityXLog.append(getPigeon2().getAngularVelocityXDevice().getValueAsDouble());
        velocityYLog.append(getPigeon2().getAngularVelocityYDevice().getValueAsDouble());
        distanceToSpeakerLog.append(distanceToSpeaker());
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(() -> getPose(), // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of
                                                                             // ChassisSpeeds to
                                                                             // drive the robot
                new HolonomicPathFollowerConfig(AutonomousConstants.TRANSLATION_PID,
                        AutonomousConstants.ROTATION_PID,
                        AutonomousConstants.MAX_MODULE_VELOCITY,
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

    public boolean isStable() {
        return (Math.abs(rotFilter.lastValue() - getPose().getRotation().getDegrees()) < 1
            && Math.abs(xFilter.lastValue() - getPose().getX()) < 1
            && Math.abs(yFilter.lastValue() - getPose().getY()) < 1);
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
        return (Math
                .abs(getPigeon2().getPitch().getValueAsDouble()) > CollisionConstants.TIP_DEADZONE
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

    public boolean isInField() {
        return field.isPoseInRegion(getPose());
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

    public void setSpeakerPose(Alliance alliance) {
        if (alliance == Alliance.Red) {
            speakerPose = VisionConstants.RED_SPEAKER_LOCATION.toTranslation2d();
        }
    }

    public double distanceToSpeaker() {
        return speakerPose.getDistance(getPose().getTranslation());
    }

    public void setDrivetrainPose(Pose2d newPose) {
        seedFieldRelative(newPose);
    }

}

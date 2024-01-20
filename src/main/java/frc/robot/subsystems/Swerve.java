package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrAinConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;
import frc.thunder.util.Pose4d;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest =
            new SwerveRequest.ApplyChassisSpeeds();
    private Limelight[] limelights;
    private boolean slowMode = false;


    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.limelights = new Limelight[] {new Limelight("limelight-front", "10.8.62.11"),
                new Limelight("limelight-back", "10.8.62.12")};

        configurePathPlanner();
    }
    // public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants...
    // modules) {
    // this(driveTrainConstants, 250, modules);
    // }

    public Limelight[] getLimelights() {
        return limelights;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume */
        updateSimState(0.02, 12);
    }

    @Override
    public void periodic() {
        for (Limelight limelight : Limelight.filterLimelights(limelights)) {
            Pose4d pose = limelight.getAlliancePose();
            addVisionMeasurement(pose.toPose2d(),
                    Timer.getFPGATimestamp() - Units.millisecondsToSeconds(pose.getLatency())
                            - VisionConstants.PROCESS_LATENCY);
        }

        LightningShuffleboard.setDouble("Swerve", "yaw", m_yawGetter.getValueAsDouble());
        LightningShuffleboard.setBool("Swerve", "Slow mode", inSlowMode());

        // System.out.println()
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of
                                                                             // ChassisSpeeds to
                                                                             // drive the robot
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

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
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
        slowMode = slow;
        System.out.println("SETTING SLOW MODE");
    }
}

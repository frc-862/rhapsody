package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;
import frc.thunder.vision.Limelight;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Limelight[] limelights;
    private boolean slowMode = false;
    private boolean disableVision = false;
    private boolean robotCentricControl = false;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, Limelights limelightSubsystem,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        this.limelights = new Limelight[]{limelightSubsystem.getStopMe()};

        configurePathPlanner();
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
        //TODO Remove the unecessary shuffleboard stuff eventually

        for (Pose4d pose : Limelight.filteredPoses(limelights)) {
            if(!disableVision){
                addVisionMeasurement(pose.toPose2d(), pose.getFPGATimestamp());
                // System.out.println("Vision Updating");
            }
            
            LightningShuffleboard.setDouble("Swerve", "PoseX", pose.toPose2d().getX());            
            LightningShuffleboard.setDouble("Swerve", "PoseY", pose.toPose2d().getY());            
            LightningShuffleboard.setDouble("Swerve", "PoseTime", pose.getFPGATimestamp()); 
        }
        
        LightningShuffleboard.setDouble("Swerve", "Timer", Timer.getFPGATimestamp());           
        LightningShuffleboard.setDouble("Swerve", "Robot Heading", getPigeon2().getAngle());
        LightningShuffleboard.setDouble("Swerve", "Odo X", getState().Pose.getX());
        LightningShuffleboard.setDouble("Swerve", "Odo Y", getState().Pose.getY());
        
        LightningShuffleboard.setBool("Swerve", "Slow mode", slowMode);
        LightningShuffleboard.setBool("Swerve", "Robot Centric", isRobotCentricControl());

        LightningShuffleboard.setBool("Sweve", "Tipped", isTipped());

        LightningShuffleboard.setDouble("Swerve", "velocity x", getPigeon2().getAngularVelocityXDevice().getValueAsDouble());
        LightningShuffleboard.setDouble("Swerve", "velocity y", getPigeon2().getAngularVelocityYDevice().getValueAsDouble());

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
        return (Math.abs(getPigeon2().getPitch().getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE || Math.abs(getPigeon2().getRoll().getValueAsDouble()) > VisionConstants.COLLISION_DEADZONE);
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
        slowMode = slow;
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
     * Returns if the robot Pose is in Wing
     * @return boolean if the robot is in the wing to start aiming STATE priming
     */
    public boolean inWing(){
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

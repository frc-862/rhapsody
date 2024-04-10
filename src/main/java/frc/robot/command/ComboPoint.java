// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;

public class ComboPoint extends Command {
    private double minPower = 0.3;

    private Swerve drivetrain;
    private XboxController driver;
    private Limelight stopMe;

    private double pidOutput;
    private double targetHeading;
    private double targetBias;
    private Translation2d targetPose;
    private Translation2d originalTargetPose;

    private PIDController pointController = VisionConstants.TAG_AIM_CONTROLLER;
    private PIDController tagController = VisionConstants.COMBO_CONTROLLER;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25, 0.5);

    private Debouncer debouncer = new Debouncer(0.2);

    private DoubleLogEntry deltaYLog;
    private DoubleLogEntry deltaXLog;
    private DoubleLogEntry targetHeadingLog;
    private DoubleLogEntry targetYLog;
    private DoubleLogEntry targetXLog;
    private DoubleLogEntry pidOutputLog;
    private DoubleLogEntry targetMinusCurrentHeadingLog;
    private DoubleLogEntry currentLog;
    private BooleanLogEntry inToleranceLog;

    /**
     * Creates a new ComboPoint.
     *
     * @param targetPose the target pose to point at
     * @param drivetrain to request movement
     * @param driver     the driver's controller, used for drive input
     * @param limelights for tag align
     */
    public ComboPoint(Translation2d targetPose, Swerve drivetrain, XboxController driver,
            Limelights limelights) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.originalTargetPose = targetPose;
        this.stopMe = limelights.getStopMe();

        addRequirements(drivetrain);

        initLogging();
    }

    public ComboPoint(double targetX, double targetY, Swerve drivetrain, XboxController driver,
            Limelights limelights) {
        this(new Translation2d(targetX, targetY), drivetrain, driver, limelights);
    }

    private boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    private Translation2d swapAlliance(Translation2d pose) {
        return new Translation2d(VisionConstants.FIELD_LIMIT.getX() - pose.getX(), pose.getY());
    }

    // private boolean inTolerance() {
    //     return Math.abs(Math.abs(targetHeading + ) - Math.abs(drivetrain.getPose().getRotation().getDegrees()))
    //             % 360 < DrivetrainConstants.ALIGNMENT_TOLERANCE;
    // }

    private boolean inTolerance() {
        return (Math.abs(targetHeading)) < VisionConstants.POINTATTAG_ALIGNMENT_TOLERANCE
                && stopMe.hasTarget();
    }

    @Override
    public void initialize() {
        pointController.enableContinuousInput(0, 360);
        pointController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
        tagController.enableContinuousInput(0, 360);
        tagController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
        if (isBlueAlliance()) {
            targetPose = originalTargetPose;
        } else {
            targetPose = swapAlliance(originalTargetPose);
        }

        // stopMe.setPipeline(VisionConstants.Pipelines.SPEAKER_PIPELINE);
        stopMe.setFiducialIDFiltersOverride(VisionConstants.SPEAKER_FILTERS);
        targetBias = 0;//5.5d;

        System.out.println("DRIVE - COMBO POINT START");
    }

    /**
     * update logging
     */
    public void initLogging() {
        DataLog log = DataLogManager.getLog();

        deltaYLog = new DoubleLogEntry(log, "/ComboPoint/Delta Y");
        deltaXLog = new DoubleLogEntry(log, "/ComboPoint/Delta X");
        targetHeadingLog = new DoubleLogEntry(log, "/ComboPoint/Target Heading");
        targetYLog = new DoubleLogEntry(log, "/ComboPoint/Target Pose Y");
        targetXLog = new DoubleLogEntry(log, "/ComboPoint/Target Pose X");
        pidOutputLog = new DoubleLogEntry(log, "/ComboPoint/Pid Output");
        targetMinusCurrentHeadingLog = new DoubleLogEntry(log, "/ComboPoint/target minus current heading");
        currentLog = new DoubleLogEntry(log, "/ComboPoint/Current");
        inToleranceLog = new BooleanLogEntry(log, "/ComboPoint/InTolerance");
    }

    public void setDebugging() {
        pointController
                .setP(LightningShuffleboard.getDouble("ComboPoint", "P", pointController.getP()));
        pointController
                .setI(LightningShuffleboard.getDouble("ComboPoint", "I", pointController.getI()));
        pointController
                .setD(LightningShuffleboard.getDouble("ComboPoint", "D", pointController.getD()));
        minPower = LightningShuffleboard.getDouble("ComboPoint", "Min Power", minPower);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose();
        var deltaX = targetPose.getX() - pose.getX();
        var deltaY = targetPose.getY() - pose.getY();

        if (stopMe.hasTarget()
                && stopMe.getPipeline() == VisionConstants.Pipelines.SPEAKER_PIPELINE) {
            targetHeading = stopMe.getTargetX() - targetBias;
            
            double currentHeading = (pose.getRotation().getDegrees() + 360) % 360;
                pidOutput = tagController.calculate(currentHeading, currentHeading - targetHeading);
        } else {
            targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 360 + 180;
            targetHeading %= 360;

            pidOutput = pointController.calculate((pose.getRotation().getDegrees() + 360) % 360,
                    targetHeading);
            // if (!inTolerance() && Math.abs(pidOutput) < minPower) {
            // pidOutput = Math.signum(pidOutput) * minPower;
            // }
        }

        double feedForwardOutput = feedforward.calculate(pidOutput);

        // setDebugging();

        drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), feedForwardOutput);

        if (!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDouble("ComboPoint", "Target Heading", targetHeading);
            LightningShuffleboard.setDouble("ComboPoint", "Current Heading",
                    drivetrain.getPose().getRotation().getDegrees());
            LightningShuffleboard.setBool("ComboPoint", "In Tolerance", inTolerance());
            LightningShuffleboard.setDouble("ComboPoint", "Raw Output (PointController)", pidOutput);
            LightningShuffleboard.setDouble("ComboPoint", "Output (FeedForward)", feedForwardOutput);
            LightningShuffleboard.setBool("ComboPoint", "HasTarget", stopMe.hasTarget());
        }

        updateLogging();
    }

    /**
     * update logging
     */
    public void updateLogging() {
        deltaYLog.append(targetPose.getY() - drivetrain.getPose().getY());
        deltaXLog.append(targetPose.getX() - drivetrain.getPose().getX());
        targetHeadingLog.append(targetHeading);
        targetYLog.append(targetPose.getY());
        targetXLog.append(targetPose.getX());
        pidOutputLog.append(pidOutput);
        targetMinusCurrentHeadingLog
                .append(Math.abs(targetHeading) - Math.abs(drivetrain.getPose().getRotation().getDegrees()));
        currentLog.append(drivetrain.getPose().getRotation().getDegrees());
        inToleranceLog.append(inTolerance());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DRIVE - COMBO POINT END");
        // stopMe.setPipeline(VisionConstants.Pipelines.TAG_PIPELINE);
        stopMe.setFiducialIDFiltersOverride(VisionConstants.ALL_TAG_FILTERS);
        if (DriverStation.isAutonomous()) {
            drivetrain.setField(0d, 0d, 0d);
        }
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(inTolerance());
    }
}

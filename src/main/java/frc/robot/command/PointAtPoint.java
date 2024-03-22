package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PointAtPoint extends Command {

    private double minPower = 0.3;

    private Swerve drivetrain;
    private XboxController driver;

    private double pidOutput;
    private double targetHeading;
    private Translation2d targetPose;
    private Translation2d originalTargetPose;

    private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

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
     * Creates a new PointAtPoint.
     *
     * @param targetPose the target pose to point at
     * @param drivetrain to request movement
     * @param driver     the driver's controller, used for drive input
     */
    public PointAtPoint(Translation2d targetPose, Swerve drivetrain, XboxController driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.originalTargetPose = targetPose;

        addRequirements(drivetrain);

        initLogging();
    }

    public PointAtPoint(double targetX, double targetY, Swerve drivetrain, XboxController driver) {
        this(new Translation2d(targetX, targetY), drivetrain, driver);
    }

    private boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    private Translation2d swapAlliance(Translation2d pose) {
        return new Translation2d(VisionConstants.FIELD_LIMIT.getX() - pose.getX(), pose.getY());
    }

    private boolean inTolerance() {
        return Math.abs(targetHeading - drivetrain.getPose().getRotation().getDegrees())
                % 360 < DrivetrainConstants.ALIGNMENT_TOLERANCE;
    }

    @Override
    public void initialize() {
        headingController.enableContinuousInput(0, 360);
        if (isBlueAlliance()) {
            targetPose = originalTargetPose;
        } else {
            targetPose = swapAlliance(originalTargetPose);
        }

        System.out.println("DRIVE - Point AT Point START");
    }

    /**
     * update logging
     */
    public void initLogging() {
        DataLog log = DataLogManager.getLog();

        deltaYLog = new DoubleLogEntry(log, "/PointAtPoint/Delta Y");
        deltaXLog = new DoubleLogEntry(log, "/PointAtPoint/Delta X");
        targetHeadingLog = new DoubleLogEntry(log, "/PointAtPoint/Target Heading");
        targetYLog = new DoubleLogEntry(log, "/PointAtPoint/Target Pose Y");
        targetXLog = new DoubleLogEntry(log, "/PointAtPoint/Target Pose X");
        pidOutputLog = new DoubleLogEntry(log, "/PointAtPoint/Pid Output");
        targetMinusCurrentHeadingLog = new DoubleLogEntry(log, "/PointAtPoint/target minus current heading");
        currentLog = new DoubleLogEntry(log, "/PointAtPoint/Current");
        inToleranceLog = new BooleanLogEntry(log, "/PointAtPoint/InTolerance");
    }


    public void setDebugging(){

        headingController.setP(LightningShuffleboard.getDouble("Point-At-Point", "P", headingController.getP()));
        headingController.setI(LightningShuffleboard.getDouble("Point-At-Point", "I", headingController.getI()));
        headingController.setD(LightningShuffleboard.getDouble("Point-At-Point", "D", headingController.getD()));
        minPower = LightningShuffleboard.getDouble("Point-At-Point", "Min Power", minPower);

    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose();
        var deltaX = targetPose.getX() - pose.getX();
        var deltaY = targetPose.getY() - pose.getY();

        targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 360 + 180;
        targetHeading %= 360;
        pidOutput = headingController.calculate((pose.getRotation().getDegrees() + 360) % 360, targetHeading);

        if (!DriverStation.isFMSAttached()){
            setDebugging();
        }

        if (!inTolerance() && Math.abs(pidOutput) < minPower) {
            pidOutput = Math.signum(pidOutput) * minPower;
        }


        drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), pidOutput);

        LightningShuffleboard.setDouble("Point-At-Point", "Target Heading", targetHeading);
        LightningShuffleboard.setDouble("Point-At-Point", "Current Heading",
                drivetrain.getPose().getRotation().getDegrees());
        LightningShuffleboard.setBool("Point-At-Point", "In Tolerance", inTolerance());

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
        targetMinusCurrentHeadingLog.append(Math.abs(targetHeading - drivetrain.getPose().getRotation().getDegrees()));
        currentLog.append(drivetrain.getPose().getRotation().getDegrees());
        inToleranceLog.append(inTolerance());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DRIVE - Point AT Point END");
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return inTolerance();
        }
        return false;
    }
}
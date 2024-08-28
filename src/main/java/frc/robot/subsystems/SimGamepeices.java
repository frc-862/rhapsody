package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PeiceSimConstants;
import frc.robot.Constants.RhapsodyPivotConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import java.util.function.DoubleSupplier;

public class SimGamepeices extends SubsystemBase{
    public Peice[] peices;
    public PivotRhapsody pivot;
    public Flywheel flywheel;
    public Collector collector;
    public Swerve drivetrain;
    public Indexer indexer;
    public Peice heldPeice;
    public Peice dispensedPeice;

    // peice class
    public class Peice {
        public Pose3d pose;
        public boolean isHeld;
        public double timeHeld;
        public double peiceNum; // for publishing

        /**
         * construct Peice in robot
         * @param pose
         * @param isHeld
         * @param peiceNum
         */
        public Peice(Pose3d pose, boolean isHeld, double peiceNum) {
            this.pose = pose;
            this.isHeld = isHeld;
            timeHeld = Utils.getCurrentTimeSeconds();
            this.peiceNum = peiceNum;
        }

        /**
         * construct a peice on the feild
         * @param pose
         * @param peiceNum
         */
        public Peice(Pose3d pose, double peiceNum) {
            this.pose = pose;
            this.isHeld = false;
            timeHeld = Utils.getCurrentTimeSeconds();
            this.peiceNum = peiceNum;
        }
    }

    /**
     * construct with peices being passed in
     * @param pivot
     * @param flywheel
     * @param collector
     * @param drivetrain
     * @param indexer
     * @param peices
     */
    public SimGamepeices(PivotRhapsody pivot, Flywheel flywheel, Collector collector, Swerve drivetrain, Indexer indexer, 
        Peice...peices) {
        this.peices = peices;
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collector = collector;
        this.drivetrain = drivetrain;
        this.indexer = indexer;

        // publish all peices to shuffleboard
        for (Peice peice : peices) {
            publish(peice);
        }
    }

    /**
     * construct with default peices
     * @param pivot
     * @param flywheel
     * @param collector
     * @param drivetrain
     * @param indexer
     */
    public SimGamepeices(PivotRhapsody pivot, Flywheel flywheel, Collector collector, Swerve drivetrain, Indexer indexer) {

        // peices on feild
        this.peices = new Peice[] {new Peice(PeiceSimConstants.C1B, 0), new Peice(PeiceSimConstants.C2B, 1), 
            new Peice(PeiceSimConstants.C3B, 2), new Peice(PeiceSimConstants.C1R, 3), new Peice(PeiceSimConstants.C2R, 4), 
            new Peice(PeiceSimConstants.C3R, 5), new Peice (PeiceSimConstants.F1B, 6), new Peice(PeiceSimConstants.F2B, 7), 
            new Peice(PeiceSimConstants.F3B, 8), new Peice(PeiceSimConstants.F4B, 9), new Peice(PeiceSimConstants.F5B, 10),};

        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collector = collector;
        this.drivetrain = drivetrain;
        this.indexer = indexer;

        // publish all peices to shuffleboard
        for (Peice peice : peices) {
            publish(peice);
        }

        // add peice in robot
        addPeice(new Peice(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), 11));
        peices[11].isHeld = true;
        heldPeice = peices[11];
    }

    public void collect(){
        // check to see if the robot already has a peice
        if (heldPeice != null) {
          return;
        }

        for (Peice peice : peices) {
            if (Math.hypot(drivetrain.getPose().getX() - peice.pose.getTranslation().getX(),
            drivetrain.getPose().getY() - peice.pose.getTranslation().getY()) // check is peice is in range
            < PeiceSimConstants.COLLECT_DISTANCE && !peice.isHeld && collector.getPower() > 0 // check is collector is on
            && drivetrain.getPose().getRotation().getDegrees() < PeiceSimConstants.COLLECT_ANGLE_MAX.getDegrees() // check if peice within the correct angle
            && drivetrain.getPose().getRotation().getDegrees() > PeiceSimConstants.COLLECT_ANGLE_MIN.getDegrees()) {
                peice.isHeld = true;
                heldPeice = peice;
                if (peice == dispensedPeice) {
                    dispensedPeice = null;
                }

                simBeamBreaks();
                break;
            }


        }
    }

    /**
     * simulate beambreaks by estimating where the peice is using subsystem speeds
     */
    public void simBeamBreaks(){
        if (heldPeice == null) {
            return;
        }

        double distanceFromStart = -0.5;
        double timeAtCollection = Utils.getCurrentTimeSeconds();

        while (distanceFromStart <= 1.5){
            double collectorSpeed = collector.getPower() * CollectorConstants.MaxAngularRate;
            double indexerSpeed = indexer.getPower() * IndexerConstants.MaxAngularRate;
            double flywheelSpeed = (flywheel.getBottomMotorRPM() + flywheel.getTopMotorRPM()) / 2 * FlywheelConstants.CIRCUMFRENCE / 60;
            double t = Utils.getCurrentTimeSeconds() - timeAtCollection;
            
            if (distanceFromStart >= -0.5 && distanceFromStart < -0.25) {
                // In Colelector only
                collector.beamBreak.setIsTriggered(true);
                distanceFromStart =+ t * collectorSpeed;
            } else if (distanceFromStart >= -0.25 && distanceFromStart < 0.5) {
                // In Collector and Indexer
                collector.beamBreak.setIsTriggered(true);
                indexer.indexerSensorEntry.setIsTriggered(true);
                distanceFromStart =+ (t * collectorSpeed + t * indexerSpeed) / 2;
            } else if (distanceFromStart >= 0.5 && distanceFromStart < 1.5) {
                // In Indexer and Flywheel
                indexer.indexerSensorEntry.setIsTriggered(false);
                collector.beamBreak.setIsTriggered(false);
                distanceFromStart =+ (t * indexerSpeed + t * flywheelSpeed) / 2;
            } else {
                // In Flywheel only
                distanceFromStart =+ t * flywheelSpeed;
                indexer.indexerSensorExit.setIsTriggered(true);
            }
        }

        shoot();
        indexer.indexerSensorExit.setIsTriggered(false);
    }

    /**
     * shoot the peice
     */
    public void shoot(){
        System.out.println("Sim shooting");
        heldPeice.isHeld = false;
        simShootTraj(heldPeice);
        heldPeice = null;
        return;
    }

    /**
     * simulate the trajectory of the peice being shot using kinematics
     * @param peice
     */
    public void simShootTraj(Peice peice){
        Pose2d initialPose = drivetrain.getPose();
        double shootAngle = pivot.getAngle();
        double shootSpeedTop = flywheel.getTopMotorRPM() * FlywheelConstants.CIRCUMFRENCE / 60;
        double shootSpeedBottom = flywheel.getBottomMotorRPM() * FlywheelConstants.CIRCUMFRENCE / 60;
        double viz = (shootSpeedTop + shootSpeedBottom) / 2 * Math.sin(shootAngle);
        double vx = (shootSpeedTop + shootSpeedBottom) / 2 * Math.cos(shootAngle);
        double dzi = pivot.simPivotPose.getY();
        double ti = Utils.getCurrentTimeSeconds();
        double t = 0;
        double dz = dzi;
        double dx = 0;
        double a = -9.8;
        double dy = 0;
        while (peice.pose.getY() > 0) {
            t = Utils.getCurrentTimeSeconds() - ti;
            // dy = viz * t + 0.5 * a * t^2 + dyi
            dz = viz * (t - ti) + 0.5 * a * Math.pow(t, 2);
            // dx = vx * t
            dx = vx * t;

            dy = dx * Math.sin(initialPose.getRotation().getDegrees()) + initialPose.getY();
            dx = dx * Math.cos(initialPose.getRotation().getDegrees()) + initialPose.getX();
            peice.pose = new Pose3d(dx, dy, dz, new Rotation3d(0, 0, initialPose.getRotation().getDegrees()));
        }
    }

    /**
     * dispense a peice from the source if teleop enabled
     */
    public void dispensePeiceFromSource(){
        if(dispensedPeice != null || DriverStation.isAutonomous() || !DriverStation.isEnabled()){
            return;
        }
        addPeice(new Peice(PeiceSimConstants.FROM_SOURCE, peices.length));
        dispensedPeice = peices[peices.length - 1];
    }

    /**
     * update the pose of the held peice
     */
    public void updateHeldPeicePose(){
        if (heldPeice == null) {
            return;
        }
        heldPeice.pose = new Pose3d(
            drivetrain.getPose().getX() + Math.cos(drivetrain.getPose().getRotation().getRadians()) 
            * (RhapsodyPivotConstants.LENGTH / 2) * Math.cos(pivot.getAngle() * 2 * Math.PI), 
            drivetrain.getPose().getY() + Math.sin(drivetrain.getPose().getRotation().getRadians())
            * (RhapsodyPivotConstants.LENGTH / 2) * Math.cos(pivot.getAngle() * 2 * Math.PI),
            (RhapsodyPivotConstants.LENGTH / 2) * Math.sin(pivot.getAngle() * 2 * Math.PI),
            new Rotation3d(0d, pivot.getAngle() * 2 * Math.PI, drivetrain.getPose().getRotation().getRadians()));
    }

    /**
     * add peice to the array of peices
     * @param peice
     */
    public void addPeice(Peice peice){
        Peice[] newPeices = new Peice[peices.length + 1]; 
        for (int i = 0; i < peices.length; i++) {
            newPeices[i] = peices[i];
        }
        newPeices[peices.length] = peice;
        peices = newPeices;
        publish(peice);
    }

    /**
     * delete peice from the array of peices
     * @param peice
     */
    public void deletePeice(Peice peice){
        Peice[] newPeices = new Peice[peices.length - 1];
        int j = 0;
        for (int i = 0; i < peices.length; i++) {
            if (peices[i] != peice) {
                newPeices[j] = peices[i];
                j++;
            }
        }
        peices = newPeices;
    }

    /**
     * publish the peice to shuffleboard
     * @param peice
     */
    public void publish(Peice peice){
        LightningShuffleboard.setDoubleArray("Peices", "peice #" + peice.peiceNum, 
            () -> new double[] {peice.pose.getX(), peice.pose.getY(), peice.pose.getZ(),
            Units.radiansToDegrees(peice.pose.getRotation().getX()), Units.radiansToDegrees(peice.pose.getRotation().getY()), 
            Units.radiansToDegrees(peice.pose.getRotation().getZ())});
    }

    
    public void periodic(){
        collect();
        dispensePeiceFromSource();
        updateHeldPeicePose();
    }
}
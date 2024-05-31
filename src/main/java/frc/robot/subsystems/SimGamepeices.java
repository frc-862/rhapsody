package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.PeiceSimConstants;
import frc.robot.Constants.RhapsodyPivotConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SimGamepeices extends SubsystemBase{
    public Peice[] peices;
    public PivotRhapsody pivot;
    public Flywheel flywheel;
    public Collector collector;
    public Swerve drivetrain;
    public Indexer indexer;
    public Peice heldPeice;
    public Peice dispensedPeice;

    public class Peice {
        public Pose3d pose;
        public boolean isHeld;
        public double timeHeld;
        public double peiceNum;

        public Peice(Pose3d pose, boolean isHeld, double peiceNum) {
            this.pose = pose;
            this.isHeld = isHeld;
            timeHeld = Utils.getCurrentTimeSeconds();
            this.peiceNum = peiceNum;
        }

        public Peice(Pose3d pose, double peiceNum) {
            this.pose = pose;
            this.isHeld = false;
            timeHeld = Utils.getCurrentTimeSeconds();
            this.peiceNum = peiceNum;
        }
    }

    public SimGamepeices(PivotRhapsody pivot, Flywheel flywheel, Collector collector, Swerve drivetrain, Indexer indexer, 
        Peice...peices) {
        this.peices = peices;
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collector = collector;
        this.drivetrain = drivetrain;
        this.indexer = indexer;

        for (Peice peice : peices) {
            publish(peice);
        }
    }

    public SimGamepeices(PivotRhapsody pivot, Flywheel flywheel, Collector collector, Swerve drivetrain, Indexer indexer) {
        this.peices = new Peice[] {new Peice(PeiceSimConstants.C1B, 0), new Peice(PeiceSimConstants.C2B, 1), 
            new Peice(PeiceSimConstants.C3B, 2), new Peice(PeiceSimConstants.C1R, 3), new Peice(PeiceSimConstants.C2R, 4), 
            new Peice(PeiceSimConstants.C3R, 5), new Peice (PeiceSimConstants.F1B, 6), new Peice(PeiceSimConstants.F2B, 7), 
            new Peice(PeiceSimConstants.F3B, 8), new Peice(PeiceSimConstants.F4B, 9), new Peice(PeiceSimConstants.F5B, 10),};

        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collector = collector;
        this.drivetrain = drivetrain;
        this.indexer = indexer;

        for (Peice peice : peices) {
            publish(peice);
        }
    }

    public void collect(){
        if (heldPeice != null) {
          return;
        }

        for (Peice peice : peices) {
            if (Math.hypot(peice.pose.getTranslation().getX(), peice.pose.getTranslation().getY()) 
            < PeiceSimConstants.COLLECT_DISTANCE && !peice.isHeld && collector.getPower() > 0
            && drivetrain.getPose().getRotation().getDegrees() < PeiceSimConstants.COLLECT_ANGLE_MAX.getDegrees()
            && drivetrain.getPose().getRotation().getDegrees() > PeiceSimConstants.COLLECT_ANGLE_MIN.getDegrees()) {
                peice.isHeld = true;
                heldPeice = peice;
                if (peice.pose == PeiceSimConstants.FROM_SOURCE) {
                    dispensedPeice = null;
                }
                break;
            }
        }
    }

    public void shoot(){
        if (heldPeice == null) {
            return;
        }
        if (indexer.getPower() > 0 && heldPeice != null && Utils.getCurrentTimeSeconds() - heldPeice.timeHeld > 1 
            && flywheel.getTopMotorRPM() > 0 && flywheel.getBottomMotorRPM() > 0) 
        {
            heldPeice.isHeld = false;
            simShootTraj(heldPeice);
            heldPeice = null;
            return;
        }
    }

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

            dx = dx * Math.cos(initialPose.getRotation().getDegrees()) + initialPose.getX();
            dy = dx * Math.sin(initialPose.getRotation().getDegrees()) + initialPose.getY();
            peice.pose = new Pose3d(dx, dy, dz, new Rotation3d(0, 0, initialPose.getRotation().getDegrees()));
        }
        // publish(peice);
    }

    public void dispensePeiceFromSource(){
        if(dispensedPeice != null || DriverStation.isAutonomous() || !DriverStation.isEnabled()){
            System.out.println("no dispensed peice");
            return;
        }
        System.out.println("dispensed peice");
        addPeice(new Peice(PeiceSimConstants.FROM_SOURCE, peices.length - 2));
        publish(peices[peices.length - 3]);
    }

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

        // publish(heldPeice);
    }

    public void addPeice(Peice peice){
        Peice[] newPeices = new Peice[peices.length + 1];
        for (int i = 0; i < peices.length; i++) {
            newPeices[i] = peices[i];
        }
        newPeices[peices.length] = peice;
        peices = newPeices;
    }

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

    public void publish(Peice peice){
        LightningShuffleboard.setDoubleArray("Peices", "peice #" + peice.peiceNum, 
            () -> new double[] {peice.pose.getX(), peice.pose.getY(), peice.pose.getZ(),
            peice.pose.getRotation().getX(), peice.pose.getRotation().getY(), peice.pose.getRotation().getZ()});
    }

    
    public void periodic(){
        shoot();
        collect();
        dispensePeiceFromSource();
        updateHeldPeicePose();
    }
}
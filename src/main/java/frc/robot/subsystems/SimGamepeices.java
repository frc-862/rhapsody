package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.PeiceSimConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SimGamepeices extends SubsystemBase{
    public Peice[] peices;
    public PivotRhapsody pivot;
    public Flywheel flywheel;
    public Collector collector;
    public Swerve drivetrain;
    public Indexer indexer;
    public Peice heldPeice;

    public class Peice {
        public Pose3d pose;
        public boolean isHeld;
        public double timeHeld;

        public Peice(Pose3d pose, boolean isHeld) {
            this.pose = pose;
            this.isHeld = isHeld;
            timeHeld = Utils.getCurrentTimeSeconds();
        }

        public Peice(Pose3d pose) {
            this.pose = pose;
            this.isHeld = false;
            timeHeld = Utils.getCurrentTimeSeconds();
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
    }

    public SimGamepeices(PivotRhapsody pivot, Flywheel flywheel, Collector collector, Swerve drivetrain, Indexer indexer) {
        this.peices = new Peice[] {new Peice(PeiceSimConstants.C1B), new Peice(PeiceSimConstants.C2B), 
            new Peice(PeiceSimConstants.C3B), new Peice(PeiceSimConstants.C1R), new Peice(PeiceSimConstants.C2R), 
            new Peice(PeiceSimConstants.C3R), new Peice (PeiceSimConstants.F1B), new Peice(PeiceSimConstants.F2B), 
            new Peice(PeiceSimConstants.F3B), new Peice(PeiceSimConstants.F4B), new Peice(PeiceSimConstants.F5B),};

        System.out.println("constructing");

        this.pivot = pivot;
        this.flywheel = flywheel;
        this.collector = collector;
        this.drivetrain = drivetrain;
        this.indexer = indexer;
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
                break;
            }
        }
    }

    public void shoot(){
        if (heldPeice == null) {
            return;
        }

        for (Peice peice : peices) {
            if (peice.isHeld) {
                peice.isHeld = false;
                heldPeice = null;
                simShootTraj(peice);
                break;
            }
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
    }

    public void dispensePeiceFromSource(){
        addPeice(new Peice(PeiceSimConstants.FROM_SOURCE));
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

    public void publish(){
        for (int i = 0; i < peices.length; i++) {
            Peice peice = peices[i];
            System.out.println("publishing");
            LightningShuffleboard.setDoubleArray("Peice", "test", () -> new double[] {12, 281, 128});
            LightningShuffleboard.setDoubleArray("Peice", "Peice " + i, () -> new double[] { 
                peice.pose.getTranslation().getX(), peice.pose.getTranslation().getY(), peice.pose.getTranslation().getZ(), 
                peice.pose.getRotation().getX(), peice.pose.getRotation().getY(), peice.pose.getRotation().getZ(), 
                peice.pose.getRotation().getZ() });
        }
    }

    
    public void simulationPeriodic(){
        System.out.println("periodictest");
        collect();

        if (indexer.getPower() > 0 && heldPeice != null && Utils.getCurrentTimeSeconds() - heldPeice.timeHeld > 1) {
            shoot();
        }

        for (Peice peice : peices) {
            if (peice.pose == PeiceSimConstants.FROM_SOURCE) {
                break;
            }
            dispensePeiceFromSource();
        }

        publish();
    }
}
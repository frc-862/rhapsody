package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.util.Pose4d;
import frc.thunder.vision.Limelight;

public class Limelights extends SubsystemBase {

    private Limelight stopMe;
    private Limelight dust;
    private Limelight champs;
    private Thread poseProducer;
    private double lastVisionRead = 0;
    private Consumer<Pose4d> visionConsumer = (Pose4d _pose) -> {
    };

    public Limelights() {
        stopMe = new Limelight("limelight-stopme", "10.8.62.11"); // LL3 Back
        dust = new Limelight("limelight-dust", "10.8.62.12"); // LL2+ Front up
        champs = new Limelight("limelight-champs", "10.8.62.13"); // LL2+ Front down (collector)

        // TODO: make actual pipelines... maybe an enum? At least use constants
        stopMe.setPipeline(0);
        dust.setPipeline(0);
        champs.setPipeline(0);

        this.poseProducer = new Thread(() -> {
            while (true) {
                try {
                    monitor(stopMe);
                    monitor(champs);
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    System.err.println("Vision loop error: " + e.toString());
                    e.printStackTrace();
                }
            }
        });
        poseProducer.start();
    }

    private void monitor(Limelight limelight) {
        if (limelight.hasTarget()) {
            Pose4d pose = limelight.getBlueAlliancePose();
            if (pose.getFPGATimestamp() > lastVisionRead && pose.trust()) {
                visionConsumer.accept(pose);
                lastVisionRead = pose.getFPGATimestamp();
            }
        }
    }

    public void setApplyVisionUpdate(Consumer<Pose4d> visionConsumer) {
        this.visionConsumer = visionConsumer;
    }

    public Limelight getStopMe() {
        return stopMe;
    }

    public Limelight getDust() {
        return dust;
    }

    public Limelight getChamps() {
        return champs;
    }

    public void setStopMePipeline(int pipeline) {
        stopMe.setPipeline(pipeline);
    }

    public void setDustPipeline(int pipeline) {
        dust.setPipeline(pipeline);
    }

    public void setChampsPipeline(int pipeline) {
        champs.setPipeline(pipeline);
    }

    public int getStopMePipeline() {
        return stopMe.getPipeline();
    }

    public int getDustPipeline() {
        return dust.getPipeline();
    }

    public int getChampsPipeline() {
        return champs.getPipeline();
    }

    @Override
    public void periodic() {
    }
}

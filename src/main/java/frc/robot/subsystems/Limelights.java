package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.thunder.vision.Limelight;

public class Limelights extends SubsystemBase {
    private Limelight stopMe;
    private Limelight dust;
    private Limelight champs;
    private Limelight champions;
    private LEDs leds;

    public Limelights(LEDs leds) {
        this.leds = leds;
        stopMe = new Limelight("limelight-pressure", "10.8.62.11");   // LL3   Back
        dust = new Limelight("limelight-dust", "10.8.62.12");           // LL2+  Front up
        champs = new Limelight("limelight-champs", "10.8.62.13"); // LL2+  Front down (collector)

        //TODO: make actual pipelines... maybe an enum? At least use constants
        stopMe.setPipeline(0);
        dust.setPipeline(0);
        champs.setPipeline(0);
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
        return  stopMe.getPipeline();
    }

    public int getDustPipeline() {
        return  dust.getPipeline();
    }

    public int getChampsPipeline() {
        return  champs.getPipeline();
    }


    @Override
    public void periodic() {
        if (stopMe.hasTarget() || champions.hasTarget()){
            leds.EnableState(LED_STATES.HAS_VISION);
        }
    }
}

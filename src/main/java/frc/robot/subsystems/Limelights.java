package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.vision.Limelight;

public class Limelights extends SubsystemBase {
    private Limelight pressure;
    private Limelight dust;
    private Limelight champions;

    public Limelights() {
        pressure = new Limelight("limelight-pressure", "10.8.62.11");   // LL3   Back
        dust = new Limelight("limelight-dust", "10.8.62.12");           // LL2+  Front up
        champions = new Limelight("limelight-champions", "10.8.62.13"); // LL2+  Front down (collector)

        //TODO: make actual pipelines... maybe an enum? At least use constants
        pressure.setPipeline(0);
        dust.setPipeline(0);
        champions.setPipeline(0);
    }

    public Limelight getPressure() {
        return pressure;
    }

    public Limelight getDust() {
        return dust;
    }

    public Limelight getChampions() {
        return champions;
    }

    public void setPressurePipeline(int pipeline) {
        pressure.setPipeline(pipeline);
    }

    public void setDustPipeline(int pipeline) {
        dust.setPipeline(pipeline);
    }

    public void setChampionsPipeline(int pipeline) {
        champions.setPipeline(pipeline);
    }

    public int getPressurePipeline() {
        return  pressure.getPipeline();
    }

    public int getDustPipeline() {
        return  dust.getPipeline();
    }

    public int getChampionsPipeline() {
        return  champions.getPipeline();
    }


    @Override
    public void periodic() {
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.Pipelines;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;
import frc.thunder.vision.Limelight;
import frc.robot.Constants.VisionConstants;

public class Limelights extends SubsystemBase {
    private Limelight shooterLimelight = new Limelight("limelight-shooter", "10.8.62.11");
    private Limelight collectorLimeLight = new Limelight("limelight-collector", "10.8.62.12");
    


    public Limelights() {
        shooterLimelight.setPipeline(VisionConstants.Pipelines.TAG_PIPELINE);
        collectorLimeLight.setPipeline(VisionConstants.Pipelines.NOTE_PIPELINE);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Limelights", "shooterLimelight target", shooterLimelight.hasTarget());
        
        LightningShuffleboard.setDouble("Limelights", "shooterLimelight tx", shooterLimelight.getTargetX());
        LightningShuffleboard.setDouble("Limelights", "shooterLimelight ty", shooterLimelight.getTargetY());
    }

    public Limelight getShooterLimeLight(){
        return shooterLimelight;
    }

    public Limelight getCollectorLimeLight(){
        return collectorLimeLight;
    }



}

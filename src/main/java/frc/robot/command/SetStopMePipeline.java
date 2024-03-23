// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelights;

public class SetStopMePipeline extends Command {

    private Limelights limelights;
    private int pipeline;

    /**
     * @param limelights
     * @param pipeline
     */
    public SetStopMePipeline(Limelights limelights, int pipeline) {
        this.limelights = limelights;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        limelights.getStopMe().setPipeline(pipeline);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

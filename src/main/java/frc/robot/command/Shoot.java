// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Shoot extends Command {
    private Flywheel flywheel;
    private Indexer indexer;

    public Shoot(Flywheel flywheel, Indexer indexer) {
        this.flywheel = flywheel;
        this.indexer = indexer;

        addRequirements(flywheel, indexer);
    }

    @Override
    public void initialize() {
        flywheel.setTargetRPM(60);
    }

    @Override
    public void execute() {
        if (flywheel.onTarget()){
            indexer.setSpeed(0.7);
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return !indexer.getEntryBeam() && !indexer.getExitBeam();
    }
}

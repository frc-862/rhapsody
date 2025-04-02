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
        switch (flywheel.getCurrentMode()) {
            case SLOW:
                flywheel.setTargetSpeed(25);
                break;
        
            case MODERATE:
                flywheel.setTargetSpeed(35);
                break;

            case FAST:
                flywheel.setTargetSpeed(50);
                break;

            default:
                break;
        }
    }

    @Override
    public void execute() {
        if (flywheel.getTopMotorSpeed() > flywheel.getTargetSpeed()-5 && flywheel.getBottomMotorSpeed() > flywheel.getTargetSpeed()-5){
            indexer.setSpeed(0.7);
        }

        LightningShuffleboard.setBool("Shoot", "End Condition", !indexer.getEntryBeam() && !indexer.getExitBeam());
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setTargetSpeed(0);
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return !indexer.getEntryBeam() && !indexer.getExitBeam();
    }
}

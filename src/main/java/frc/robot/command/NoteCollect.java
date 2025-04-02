// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.io.Console;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class NoteCollect extends Command {
    private Collector collector;
    private Indexer indexer;

    public NoteCollect(Collector collector, Indexer indexer){
        this.collector = collector;
        this.indexer = indexer;

        addRequirements(collector, indexer);
    }

    @Override
    public void initialize(){
        collector.setSpeed(0.5);
        indexer.setSpeed(0.5);
    }

    @Override
    public void execute(){
        if (indexer.getEntryBeam()){
            collector.setSpeed(0.2);
            indexer.setSpeed(0.2);
        }
    }

    @Override
    public void end(boolean interrupted){
        collector.setSpeed(0);
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return indexer.getEntryBeam() && indexer.getExitBeam();
    }
}

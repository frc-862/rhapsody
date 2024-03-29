// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class CollectedNote extends Command {

    private final Indexer indexer;

    private final double timeout;

    private final Timer timer = new Timer();

    /**
     * Creates a new CollectedNote.
     * @param timeout in seconds, if a note is not collected the command will finish
     *                in this many seconds
     * @param indexer not required, but used to access beam break sensors
     */
    public CollectedNote(double timeout, Indexer indexer) {
        this.timeout = timeout;
        this.indexer = indexer;
    }

    /**
     * Creates a new CollectedNote.
     * @param indexer not required, but used to read sensors
     */
    public CollectedNote(Indexer indexer) {
        this(0.5, indexer);
    }

    @Override
    public void initialize() {
        indexer.clearHasShot();
        timer.restart();
    }

    // This command will end after you shoot a note, or after
    // after timeout seconds if you do not possess a note.
    // It can be used in auton to skip to the next note if we
    // fail to collect
    @Override
    public boolean isFinished() {
        return indexer.hasShot() || (!indexer.hasNote() && timer.hasElapsed(timeout));
    }
}

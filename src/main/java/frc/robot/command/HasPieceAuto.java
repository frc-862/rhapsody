package frc.robot.command;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class HasPieceAuto extends Command {

    private Indexer indexer;
    private Debouncer hasNoteDebouncer = new Debouncer(0.1, DebounceType.kFalling);

    /**
     * Creates a new HasPieceAuto
     * A race condition to move to next collect if we miss the piece
     * Ends if we do not have a piece
     * @param indexer subsystem
     */
    public HasPieceAuto(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        System.out.println("AUTO - Has Piece INIT");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AUTO - Has Piece END");
    }

    @Override
    public boolean isFinished() {
        return !hasNoteDebouncer.calculate(indexer.hasNote());
    }
}

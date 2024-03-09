package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShuffleboardPeriodicConstants;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Pair;

public class Indexer extends SubsystemBase {

    private Collector collector;

    private ThunderBird indexerMotor;
    private DigitalInput indexerSensorEntry = new DigitalInput(DIO.INDEXER_ENTER_BEAMBREAK);
    private DigitalInput indexerSensorExit = new DigitalInput(DIO.INDEXER_EXIT_BEAMBREAK);

    private int exitIndexerIteration = 0;

    private double timeLastTriggered = 0d;

    private double targetPower = 0;

    private PieceState currentState = PieceState.NONE;
    private boolean didShoot = false;

    private LightningShuffleboardPeriodic periodicShuffleboard;

    public Indexer(Collector collector) {
        this.collector = collector;

        indexerMotor = new ThunderBird(CAN.INDEXER_MOTOR, CAN.CANBUS_FD,
                IndexerConstants.MOTOR_INVERT, IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT,
                IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);

        indexerMotor.applyConfig();
        initLogging();
    }

    @SuppressWarnings("unchecked")
    private void initLogging() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Indexer", ShuffleboardPeriodicConstants.DEFAULT_SHUFFLEBOARD_PERIOD,
                new Pair<String, Object>("Indexer Power", (DoubleSupplier) () -> indexerMotor.get()),
                new Pair<String, Object>("Entry Beam Break", (Supplier<Boolean>) () -> getEntryBeamBreakState()),
                new Pair<String, Object>("Exit Beam Break", (Supplier<Boolean>) () -> getExitBeamBreakState()),
                new Pair<String, Object>("Piece State", (Supplier<String>) () -> getPieceState().toString()),
                new Pair<String, Object>("Has shot", (Supplier<Boolean>) () -> hasShot()),
                new Pair<String, Object>("Is Exiting", (Supplier<Boolean>) () -> isExiting()),
                new Pair<String, Object>("Has Piece", (Supplier<Boolean>) () -> getPieceState() == PieceState.IN_COLLECT));
    }

    /**
     * Get current state of piece
     * 
     * @return current state of piece
     */
    public PieceState getPieceState() {
        return currentState;
    }

    /**
     * Set the current state of the piece
     * 
     * @param state new state of piece
     */
    public void setPieceState(PieceState state) {
        currentState = state;
    }

    /**
     * Set raw power to the indexer motor
     * 
     * @param power
     */
    public void setPower(double power) {
        targetPower = power;
        indexerMotor.set(power);
    }

    /**
     * Index up
     */
    public void indexUp() {
        setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
    }

    /**
     * Index down
     */
    public void indexDown() {
        setPower(-IndexerConstants.INDEXER_DEFAULT_POWER);
    }

    /**
     * Stop the indexer
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * Gets the current beam brake state
     * 
     * @return entry beambreak state
     */
    public boolean getEntryBeamBreakState() {
        return !indexerSensorEntry.get();
    }

    /**
     * Gets the current beam brake state
     * 
     * @return exit beambreak state
     */
    public boolean getExitBeamBreakState() {
        return !indexerSensorExit.get();
    }

    /**
     * @return true if piece is exiting the indexer
     */
    public boolean isExiting() {
        return exitIndexerIteration >= 1;
    }

    /**
     * Will return true after shooting (or really anytime we no longer have a note,
     * after previously having one)
     * 
     * @return boolean
     */
    public boolean hasShot() {
        return didShoot;
    }

    /**
     * Has shot flag stays on until
     * cleared, will be false on
     * robot init
     */
    public void clearHasShot() {
        didShoot = false;
    }

    @Override
    public void periodic() {
        // Update piece state based on beambreaks
        if (getExitBeamBreakState()) {
            exitIndexerIteration++;
            if (exitIndexerIteration >= 3) {
                setPieceState(PieceState.IN_INDEXER);
            }
        } else if (getEntryBeamBreakState()) {
            setPieceState(PieceState.IN_PIVOT);
        } else if (collector.getEntryBeamBreakState()) {
            timeLastTriggered = Timer.getFPGATimestamp();
            setPieceState(PieceState.IN_COLLECT);
        } else if (Timer.getFPGATimestamp() - timeLastTriggered <= 1) {
            setPieceState(PieceState.IN_COLLECT);
        } else {
            didShoot = didShoot || hasNote();
            setPieceState(PieceState.NONE);
        }

        // reset exitIndexerIteration
        if (!getEntryBeamBreakState()) {
            exitIndexerIteration = 0;
        }
    }

    public boolean hasNote() {
        return getPieceState() != PieceState.NONE;
    }
}

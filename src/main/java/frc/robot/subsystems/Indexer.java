package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {

    private Collector collector;

    private ThunderBird indexerMotor;
    private DigitalInput indexerSensorEntry = new DigitalInput(DIO.INDEXER_ENTER_BEAMBREAK);
    private DigitalInput indexerSensorExit = new DigitalInput(DIO.INDEXER_EXIT_BEAMBREAK);

    private PieceState currentState = PieceState.NONE;

    public Indexer(Collector collector) {
        this.collector = collector;

        indexerMotor = new ThunderBird(CAN.INDEXER_MOTOR, CAN.CANBUS_FD,
                IndexerConstants.MOTOR_INVERT, IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT,
                IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);

        initLogging();
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Indexer", "Indexer Power", () -> indexerMotor.get());

        LightningShuffleboard.setBoolSupplier("Indexer", "Entry Beam Break", () -> getEntryBeamBreakState());
        LightningShuffleboard.setBoolSupplier("Indexer", "Exit Beam Break", () -> getExitBeamBreakState());
        LightningShuffleboard.setStringSupplier("Indexer", "Piece State", () -> getPieceState().toString());
        LightningShuffleboard.setBoolSupplier("Indexer", "Has shot", () -> hasShot());
    }

    /**
     * Get current state of piece
     * @return current state of piece
     */
    public PieceState getPieceState() {
        return currentState;
    }

    /**
     * Set the current state of the piece
     * @param state new state of piece
     */
    public void setPieceState(PieceState state) {
        currentState = state;
    }

    /**
     * Set raw power to the indexer motor
     * @param power
     */
    public void setPower(double power) {
        indexerMotor.set(power);
    }

    /**
     * Index in
     */
    public void indexIn() {
        setPower(IndexerConstants.INDEXER_DEFAULT_POWER);
    }

    /**
     * Index out
     */
    public void indexOut() {
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
     * @return entry beambreak state
     */
    public boolean getEntryBeamBreakState() {
        return !indexerSensorEntry.get();
    }

    /**
     * Gets the current beam brake state
     * @return exit beambreak state
     */
    public boolean getExitBeamBreakState() {
        return !indexerSensorExit.get();
    }

    /**
     * TO BE IMPLEMENTED
     * @return boolean
     */
    public boolean hasShot() {
        return false; // TODO add actual logic
    }

    @Override
    public void periodic() {
        // Update piece state based on beambreaks
        if (getExitBeamBreakState()) {
            setPieceState(PieceState.IN_INDEXER);
        } else if (getEntryBeamBreakState()) {
            setPieceState(PieceState.IN_PIVOT);
        } else if (collector.getEntryBeamBreakState()) {
            setPieceState(PieceState.IN_COLLECT);
        } else {
            setPieceState(PieceState.NONE);
        }
    }
}

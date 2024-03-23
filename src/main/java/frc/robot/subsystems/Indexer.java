package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.PieceState;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {

    private Collector collector;

    private ThunderBird motor;
    private DigitalInput indexerSensorEntry = new DigitalInput(DIO.INDEXER_ENTER_BEAMBREAK);
    private DigitalInput indexerSensorExit = new DigitalInput(DIO.INDEXER_EXIT_BEAMBREAK);

    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);

    private double timeLastTriggered = 0d;

    private double targetPower = 0;

    private PieceState currentState = PieceState.NONE;
    private boolean didShoot = false;

    private Debouncer entryDebouncer = new Debouncer(0.05);
    private Debouncer exitDebouncer = new Debouncer(0.05);

    private DoubleLogEntry indexerPowerLog;
    private DoubleLogEntry indexerTargetPowerLog;
    private BooleanLogEntry entryBeamBreakLog;
    private BooleanLogEntry exitBeamBreakLog;
    private StringLogEntry pieceStateLog;
    private BooleanLogEntry hasShotLog;
    private BooleanLogEntry isExitingLog;
    private BooleanLogEntry hasPieceLog;

    public Indexer(Collector collector) {
        this.collector = collector;

        motor = new ThunderBird(CAN.INDEXER_MOTOR, CAN.CANBUS_FD,
                IndexerConstants.MOTOR_INVERT, IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT,
                IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);
        motor.configSupplyLimit(0d);
        motor.configStatorLimit(80d);

        motor.applyConfig();

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        indexerPowerLog = new DoubleLogEntry(log, "/Indexer/Power");
        indexerTargetPowerLog = new DoubleLogEntry(log, "/Indexer/TargetPower");
        entryBeamBreakLog = new BooleanLogEntry(log, "/Indexer/EntryBeamBreak");
        exitBeamBreakLog = new BooleanLogEntry(log, "/Indexer/ExitBeamBreak");
        pieceStateLog = new StringLogEntry(log, "/Indexer/PieceState");
        hasShotLog = new BooleanLogEntry(log, "/Indexer/HasShot");
        isExitingLog = new BooleanLogEntry(log, "/Indexer/IsExiting");
        hasPieceLog = new BooleanLogEntry(log, "/Indexer/HasPiece");

        LightningShuffleboard.setDoubleSupplier("Indexer", "Power", () -> motor.get());

        LightningShuffleboard.setBoolSupplier("Indexer", "EntryBeamBreak", () -> getEntryBeamBreakState());
        LightningShuffleboard.setBoolSupplier("Indexer", "ExitBeamBreak", () -> getExitBeamBreakState());

        LightningShuffleboard.setStringSupplier("Indexer", "PieceState", () -> getPieceState().toString());

        LightningShuffleboard.setBoolSupplier("Indexer", "HasShot", () -> hasShot());
        LightningShuffleboard.setBoolSupplier("Indexer", "IsExiting", () -> isExiting());
        LightningShuffleboard.setBoolSupplier("Indexer", "HasPiece", () -> hasNote());
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
        motor.setControl(dutyCycleControl.withOutput(power));
    }

    /**
     * Get the current power of the indexer motor
     *
     * @return current power of the indexer motor
     */
    public double getPower() {
        return motor.get();
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
        if (Constants.isMercury()) {
            return entryDebouncer.calculate(!indexerSensorEntry.get());
        }
        return entryDebouncer.calculate(indexerSensorEntry.get());
    }

    /**
     * Gets the current beam brake state
     *
     * @return exit beambreak state
     */
    public boolean getExitBeamBreakState() {
        return exitDebouncer.calculate(!indexerSensorExit.get());
    }

    /**
     * @return true if piece is exiting the indexer
     */
    public boolean isExiting() {
        return getExitBeamBreakState() && getPieceState() == PieceState.IN_INDEXER;
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

    /**
     * Get the current power of the indexer motor
     *
     * @return current power of the indexer motor
     */
    public double getIndexerPower() {
        return motor.get();
    }

    @Override
    public void periodic() {
        // Update piece state based on beambreaks
        if (getExitBeamBreakState()) {
            setPieceState(PieceState.IN_INDEXER);
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

        updateLogging();
    }

    /**
     * update logging
     */
    public void updateLogging() {
        indexerPowerLog.append(motor.get());
        indexerTargetPowerLog.append(targetPower);
        entryBeamBreakLog.append(getEntryBeamBreakState());
        exitBeamBreakLog.append(getExitBeamBreakState());
        pieceStateLog.append(getPieceState().toString());
        hasShotLog.append(hasShot());
        isExitingLog.append(isExiting());
        hasPieceLog.append(hasNote());
    }

    public boolean hasNote() {
        return getPieceState() != PieceState.NONE;
    }
}

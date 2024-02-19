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

    private ThunderBird indexerMotor;
    private DigitalInput indexerSensorEntry = new DigitalInput(DIO.INDEXER_ENTER_BEAMBREAK);
    private DigitalInput indexerSensorExit = new DigitalInput(DIO.INDEXER_EXIT_BEAMBREAK);

    private PieceState currentState = PieceState.NONE;

    public Indexer() {
        indexerMotor = new ThunderBird(CAN.INDEXER_MOTOR, CAN.CANBUS_FD,
                IndexerConstants.MOTOR_INVERT, IndexerConstants.MOTOR_STATOR_CURRENT_LIMIT,
                IndexerConstants.INDEXER_MOTOR_BRAKE_MODE);

        initLogging();
    }

    private void initLogging() {
        LightningShuffleboard.setDoubleSupplier("Indexer", "Indexer Power", () -> indexerMotor.get());

        LightningShuffleboard.setBoolSupplier("Indexer", "Entry Beam Break", () -> getEntryBeamBreakState());
        LightningShuffleboard.setBoolSupplier("Indexer", "Exit Beam Break", () -> getExitBeamBreakState());
        LightningShuffleboard.setStringSupplier("Indexer", "Piece State", () -> getPieceState().toString()); // TODO test 
        LightningShuffleboard.setBoolSupplier("Indexer", "Has shot", () -> hasShot());
    }

    public PieceState getPieceState() {
        return currentState;
    }

    public void setPieceState(PieceState state) {
        currentState = state;
    }

    public void setPower(double power) {
        indexerMotor.set(power);
    }

    public void indexIn() {
        indexerMotor.set(IndexerConstants.INDEXER_DEFAULT_POWER);
    }

    public void indexOut() {
        indexerMotor.set(-IndexerConstants.INDEXER_DEFAULT_POWER);
    }

    public void stop() {
        indexerMotor.set(0d);
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

    public boolean hasShot() {
        return false; // TODO add actual logic
    }
}

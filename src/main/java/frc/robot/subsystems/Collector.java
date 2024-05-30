package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Collector extends SubsystemBase {

    // sim collector
    @SuppressWarnings({ "rawtypes", "unchecked" })
    private LinearSystemSim collectorSim = new LinearSystemSim(LinearSystemId.identifyVelocitySystem(
        CollectorConstants.MOTOR_KV, CollectorConstants.SIM_KA));

    private PIDController collectorController = new PIDController(CollectorConstants.SIM_KP, 
        CollectorConstants.SIM_KI, CollectorConstants.SIM_KD);

    // Declare collector hardware
    private ThunderBird motor;
    private DigitalInput beamBreak;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(
            0, 0, true, CollectorConstants.MOTOR_KV,
            0, false, false, false);

    private boolean hasPiece;
    private double targetPower;
    private double pidOutPut;

    private DoubleLogEntry collectorPowerLog;
    private BooleanLogEntry hasPieceLog;

    private Debouncer entryDebouncer = new Debouncer(0.4);

    public Collector() {
        motor = new ThunderBird(
                CAN.COLLECTOR_MOTOR, CAN.CANBUS_FD,
                CollectorConstants.COLLECTOR_MOTOR_INVERTED,
                CollectorConstants.COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT,
                CollectorConstants.COLLECTOR_MOTOR_BRAKE_MODE);

        motor.configPIDF(0, CollectorConstants.MOTOR_KP, CollectorConstants.MOTOR_KI,
                CollectorConstants.MOTOR_KD, CollectorConstants.MOTOR_KS, CollectorConstants.MOTOR_KV);

        beamBreak = new DigitalInput(DIO.COLLECTOR_BEAMBREAK);
        motor.applyConfig();

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        collectorPowerLog = new DoubleLogEntry(log, "/Collector/Power");
        hasPieceLog = new BooleanLogEntry(log, "/Collector/HasPiece");
    }

    /**
     * Entrance of Collector Beam Break
     *
     * @return When an object is present, returns true, otherwise returns false
     */
    public boolean getEntryBeamBreakState() {
        if (Constants.IS_MERCURY) {
            return entryDebouncer.calculate(!beamBreak.get());
        }
        return entryDebouncer.calculate(beamBreak.get());
    }

    /**
     * Sets the power of both collector motors
     *
     * @param power Double value from -1.0 to 1.0 (positive collects inwards)
     */
    public void setPower(double power) {
        // Convert from -1,1 to RPS
        power = power * 100;
        motor.setControl(velocityVoltage.withVelocity(power));
        targetPower = power;
    }

    /**
     * gets the current power of the collector motor
     * 
     * @return the current power of the collector motor
     */
    public double getPower() {
        if (RobotBase.isSimulation()){
            return collectorSim.getOutput(0);
        }
        return motor.get();
    }

    @Override
    public void periodic() {
        // tells robot if we have a piece in collector
        hasPiece = getEntryBeamBreakState();
        updateLogging();
    }

    @Override
    public void simulationPeriodic() {
        pidOutPut = -collectorController.calculate(targetPower, collectorSim.getOutput(0));

        collectorSim.setInput(pidOutPut * 12);
        collectorSim.update(0.01);
    }

    /**
     * update logging
     */
    public void updateLogging() {
        collectorPowerLog.append(motor.get());
        hasPieceLog.append(hasPiece());

        if(!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDouble("Collector", "Collector power", getPower());
            LightningShuffleboard.setDouble("Collector", "Collector target power", targetPower);
            LightningShuffleboard.setDouble("Collector", "pidOutPut", pidOutPut);
        }
    }

    /**
     * Has piece
     *
     * @return boolean, true if collector has piece
     */
    public boolean hasPiece() {
        return hasPiece;
    }

    /**
     * Stops the collector
     */
    public void stop() {
        setPower(0d);
    }
}

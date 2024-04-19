package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Flywheel extends SubsystemBase {
    private ThunderBird topMotor;
    private ThunderBird bottomMotor;

    private FlywheelSim flywheelSimTop = new FlywheelSim(
        DCMotor.getFalcon500(1), FlywheelConstants.GEAR_RATIO, FlywheelConstants.MOMENT_OF_INERTIA);

    private FlywheelSim flywheelSimBottom = new FlywheelSim(
        DCMotor.getFalcon500(1), FlywheelConstants.GEAR_RATIO, FlywheelConstants.MOMENT_OF_INERTIA);

    private final VelocityVoltage topRPMPID = new VelocityVoltage(0);
    private final VelocityVoltage bottomRPMPID = new VelocityVoltage(0);

    private double topTargetRPS = 0;
    private double bottomTargetRPS = 0;
    private double bias = 0;
    private boolean coast = false;
    private boolean kamaDone = false;

    private DoubleLogEntry topRPMLog;
    private DoubleLogEntry bottomRPMLog;
    private DoubleLogEntry topTargetRPMLog;
    private DoubleLogEntry bottomTargetRPMLog;
    private BooleanLogEntry topOnTargetLog;
    private BooleanLogEntry bottomOnTargetLog;
    private DoubleLogEntry topPowerLog;
    private DoubleLogEntry bottomPowerLog;
    private DoubleLogEntry biasLog;

    public Flywheel() {
        boolean topMotorInvert = FlywheelConstants.MOTOR_TOP_INVERT_Rhapsody;

        if(Constants.IS_MERCURY) {
            topMotorInvert = FlywheelConstants.MOTOR_TOP_INVERT_Mercury;
        }

        /* TEST after kettering basic stuff for now */
        topMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD,
            topMotorInvert, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
            FlywheelConstants.MOTOR_BRAKE_MODE);
        bottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD,
            FlywheelConstants.MOTOR_BOTTOM_INVERT, FlywheelConstants.MOTOR_STATOR_CURRENT_LIMIT,
            FlywheelConstants.MOTOR_BRAKE_MODE);
        topMotor.configPIDF(0, FlywheelConstants.TOP_0_MOTOR_KP, FlywheelConstants.TOP_0_MOTOR_KI,
            FlywheelConstants.TOP_0_MOTOR_KD, FlywheelConstants.TOP_0_MOTOR_KS, FlywheelConstants.TOP_0_MOTOR_KV);
        bottomMotor.configPIDF(0, FlywheelConstants.BOTTOM_0_MOTOR_KP, FlywheelConstants.BOTTOM_0_MOTOR_KI,
            FlywheelConstants.BOTTOM_0_MOTOR_KD, FlywheelConstants.BOTTOM_0_MOTOR_KS, FlywheelConstants.BOTTOM_0_MOTOR_KV);

            topMotor.configPIDF(1, FlywheelConstants.TOP_1_MOTOR_KP, FlywheelConstants.TOP_1_MOTOR_KI,
            FlywheelConstants.TOP_1_MOTOR_KD, FlywheelConstants.TOP_1_MOTOR_KS, FlywheelConstants.TOP_1_MOTOR_KV);
        bottomMotor.configPIDF(1, FlywheelConstants.BOTTOM_1_MOTOR_KP, FlywheelConstants.BOTTOM_1_MOTOR_KI,
            FlywheelConstants.BOTTOM_1_MOTOR_KD, FlywheelConstants.BOTTOM_1_MOTOR_KS, FlywheelConstants.BOTTOM_1_MOTOR_KV);

        topMotor.applyConfig();
        bottomMotor.applyConfig();

        initLogging();
    }

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        topRPMLog = new DoubleLogEntry(log, "/Flywheel/TopRPM");
        bottomRPMLog = new DoubleLogEntry(log, "/Flywheel/BottomRPM");
        topTargetRPMLog = new DoubleLogEntry(log, "/Flywheel/TopTargetRPM");
        bottomTargetRPMLog = new DoubleLogEntry(log, "/Flywheel/BottomTargetRPM");
        topOnTargetLog = new BooleanLogEntry(log, "/Flywheel/TopOnTarget");
        bottomOnTargetLog = new BooleanLogEntry(log, "/Flywheel/BottomOnTarget");
        topPowerLog = new DoubleLogEntry(log, "/Flywheel/TopPower");
        bottomPowerLog = new DoubleLogEntry(log, "/Flywheel/BottomPower");
        biasLog = new DoubleLogEntry(log, "/Flywheel/Bias");

		if (!DriverStation.isFMSAttached()) {
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Top RPM", () -> getTopMotorRPM());
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Bottom RPM", () -> getBottomMotorRPM());

            LightningShuffleboard.setDoubleSupplier("Flywheel", "Top Target RPM", () -> topMotorTargetRPM());
            LightningShuffleboard.setDoubleSupplier("Flywheel", "Bottom Target RPM", () -> bottomMotorTargetRPM());

            LightningShuffleboard.setBoolSupplier("Flywheel", "Top on Target", () -> topMotorRPMOnTarget());
            LightningShuffleboard.setBoolSupplier("Flywheel", "Bottom on Target", () -> bottomMotorRPMOnTarget());

            LightningShuffleboard.setDoubleSupplier("Flywheel", "Bias", () -> getBias());
        }
    }

    @Override
    public void periodic() {
        if (coast) {
            bottomMotor.set(0d);
            topMotor.set(0d);
        } else {
            applyPowerTop(topTargetRPS + bias);
            applyPowerBottom(bottomTargetRPS + bias);
        }

        updateLogging();
    }

    public void simulationPeriodic() {
        flywheelSimTop.setInput(topMotor.get() * 12);
        flywheelSimTop.update(0.01);

        flywheelSimBottom.setInput(bottomMotor.get() * 12);
        flywheelSimBottom.update(0.01);
    }

    /**
     * update logging
     */
    public void updateLogging() {
        topRPMLog.append(getTopMotorRPM());
        bottomRPMLog.append(getBottomMotorRPM());
        topTargetRPMLog.append(topMotorTargetRPM());
        bottomTargetRPMLog.append(bottomMotorTargetRPM());
        topOnTargetLog.append(topMotorRPMOnTarget());
        bottomOnTargetLog.append(bottomMotorRPMOnTarget());
        topPowerLog.append(topMotor.get());
        bottomPowerLog.append(bottomMotor.get());
        biasLog.append(getBias());
    }

    /**
     * Sets the RPM of all flywheel motors
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        coast(false);
        topTargetRPS = rpm / 60d;
        bottomTargetRPS = rpm / 60d;
    }

    /**
     * Sets the RPM of top flywheel
     * @param rpm RPM of the flywheel
     */
    public void setTopMotorRPM(double rpm) {
        coast(false);
        topTargetRPS = rpm / 60d;
    }

    /**
     * Sets the RPM of bottom flywheel
     * @param rpm RPM of the flywheel
     */
    public void setBottomMotorRPM(double rpm) {
        coast(false);
        bottomTargetRPS = rpm / 60d;
    }

    /**
     * @return The current RPM of flywheel top
     */
    public double getTopMotorRPM() {
        return (topMotor.getVelocity().getValue() * 60d);
    }

    /**
     * @return The current RPM of flywheel bottom
     */
    public double getBottomMotorRPM() {
        return (bottomMotor.getVelocity().getValue() * 60d);
    }

    /**
     * @return Whether or not top flywheel is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean topMotorRPMOnTarget() {
        return Math.abs(getTopMotorRPM() - (topTargetRPS * 60d)) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not bottom flywheel is on target, within FlywheelConstants.RPM_TOLERANCE
     */
    public boolean bottomMotorRPMOnTarget() {
        return Math.abs(getBottomMotorRPM() - (bottomTargetRPS * 60d)) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not all flywheel motors are on target, within RPM tolerance
     */
    public boolean allMotorsOnTarget() {
        return (topMotorRPMOnTarget() && bottomMotorRPMOnTarget());
    }

    /**
     * @return the top motor's target RPM
     */
    public double topMotorTargetRPM() {
        return topTargetRPS * 60d;
    }

    /**
     * @return the bottom motor's target RPM
     */
    public double bottomMotorTargetRPM() {
        return bottomTargetRPS * 60d;
    }

    /**
     * Sets the voltage to a small amount so the flywheel coasts to a stop
     * @param coast Whether or not to coast the flywheel
     */
    public void coast(boolean coast) {
        this.coast = coast;
    }

    public void stop() {
        setAllMotorsRPM(0);
    }

    /**
     * @return The bias to add to the target RPM of the flywheel
     */
    public double getBias() {
        return bias;
    }

    /**
     * Increases the bias of the Flywheel by set amount
     */
    public void increaseBias() {
        bias += FlywheelConstants.BIAS_INCREMENT;
    }

    /**
     * Decreases the bias of the Flywheel by set amount
     */
    public void decreaseBias() {
        bias -= FlywheelConstants.BIAS_INCREMENT;
    }

    /**
     * Resets the bias of the Flywheel
     */
    public void resetBias() {
        bias = 0;
    }
    
    public void startKama() {
        kamaDone = false;
    }

    public void endKama() {
        kamaDone = true;
    }

    public boolean getKama() {
        return kamaDone;
    }

    /**
     * Sets target RPS to the bottom motor, using the proper slots and FOC
     * @param targetRPS
     */
    private void applyPowerTop(double targetRPS) {
        if(targetRPS > 100) {
            topMotor.setControl(topRPMPID.withVelocity(targetRPS).withEnableFOC(false).withSlot(0));
        // } else if (targetRPS > 50){
        //     topMotor.setControl(topRPMPID.withVelocity(targetRPS).withEnableFOC(true).withSlot(1));
        } else {
            topMotor.setControl(topRPMPID.withVelocity(targetRPS).withEnableFOC(true).withSlot(0));
        }
    }

    /**
     * Sets target RPS to the bottom motor, using the proper slots and FOC
     * @param targetRPS
     */
    private void applyPowerBottom(double targetRPS) {
        if(targetRPS > 100) {
            bottomMotor.setControl(bottomRPMPID.withVelocity(targetRPS).withEnableFOC(false).withSlot(0));
        // } else if (targetRPS > 50){
        //     bottomMotor.setControl(bottomRPMPID.withVelocity(targetRPS).withEnableFOC(true).withSlot(1));
        } else {
            bottomMotor.setControl(bottomRPMPID.withVelocity(targetRPS).withEnableFOC(true).withSlot(0));
        }
    }
}

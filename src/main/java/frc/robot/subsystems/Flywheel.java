package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Robot;
import frc.robot.Constants.FlywheelConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.thunder.config.FalconConfig;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.FalconTuner;

public class Flywheel extends SubsystemBase {
    private ThunderBird shooterTopMotor; // TODO figure out which is top vs bottom
    private ThunderBird shooterBottomMotor;

    private final VelocityVoltage topPID = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage bottomPID = new VelocityVoltage(0).withSlot(0);
    private double topTarget = 0;
    private double bottomTarget = 0;
    private double bias = 0;

    // flywheel simulation variables
    private FlywheelSim topSim = new FlywheelSim(DCMotor.getFalcon500(1), FlywheelConstants.FLYWHEEL_GEARING, 0.01936);
    private FlywheelSim bottomSim = new FlywheelSim(DCMotor.getFalcon500(1), FlywheelConstants.FLYWHEEL_GEARING, 0.01936);
    private TalonFXSimState topSimState;
    private TalonFXSimState bottomSimState;

    private FalconTuner topTuner;
    private FalconTuner bottomTuner;

    public Flywheel() {
        shooterTopMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_TOP, CAN.CANBUS_FD, FlywheelConstants.FLYWHEEL_MOTOR_TOP_INVERT, 
            FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_BRAKE_MODE);
        shooterBottomMotor = new ThunderBird(CAN.FLYWHEEL_MOTOR_BOTTOM, CAN.CANBUS_FD, FlywheelConstants.FLYWHEEL_MOTOR_BOTTOM_INVERT,
            FlywheelConstants.FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT, FlywheelConstants.FLYWHEEL_MOTOR_BRAKE_MODE);
        
        
        shooterTopMotor.configPIDF(0,FlywheelConstants.FLYWHEEL_MOTOR_KP,
            FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
            FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);

        shooterBottomMotor.configPIDF(0,FlywheelConstants.FLYWHEEL_MOTOR_KP,
            FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD,
            FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV);

        // note - if you wanna tune these together, you should be able to set the tab to the same name and it'll read the same values for both instances
        // topTuner = new FalconTuner(shooterTopMotor, "Flywheel Top", this::setTopMoterRPM, targetRPM);
        // bottomTuner = new FalconTuner(shooterBottomMotor, "Flywheel Bottom", this::setBottomMoterRPM, targetRPM);


        if(Robot.isSimulation()) {
            topSimState = shooterTopMotor.getSimState();
            bottomSimState = shooterBottomMotor.getSimState();
        }

    }
            
    public void simulationPeriodic() {
        topSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        bottomSimState.setSupplyVoltage(RobotController.getBatteryVoltage());


        // update flywheel states as per CTRE docs
        topSim.setInputVoltage(topSimState.getMotorVoltage());
        topSim.update(0.02); 

        bottomSim.setInputVoltage(bottomSimState.getMotorVoltage());
        bottomSim.update(0.02);

        topSimState.setRotorVelocity(topSim.getAngularVelocityRPM());
        bottomSimState.setRotorVelocity(bottomSim.getAngularVelocityRPM());

        // simulation shuffleboards FOR TESTING
        // setAllMotorsRPM(LightningShuffleboard.getDouble("shooterSim", "rpm targ", 50));

        // flywheelTuner1.update();
        // flywheelTuner2.update();

        LightningShuffleboard.setDouble("shooterSim", "top rpm", getTopMotorRPM());
        LightningShuffleboard.setDouble("shooterSim", "bottom rpm", getBottomMotorRPM());

        // LightningShuffleboard.setDouble("shooterSim", "voltage", topState.getMotorVoltage());
        // LightningShuffleboard.setDouble("shooterSim", "simRPM", topSim.getAngularVelocityRPM());
        // LightningShuffleboard.setDouble("shooterSim", "curent", topState.getSupplyCurrent());
    }

    /**
     * Sets the RPM of all flywheel motors
     * 
     * @param rpm RPM of the flywheel
     */
    public void setAllMotorsRPM(double rpm) {
        topTarget = rpm;
        bottomTarget = rpm;
        shooterTopMotor.setControl(topPID.withVelocity(rpm));
        shooterBottomMotor.setControl(bottomPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 1
     * 
     * @param rpm RPM of the flywheel
     */
    public void setTopMoterRPM(double rpm) {
        topTarget = rpm;
        shooterTopMotor.setControl(topPID.withVelocity(rpm));
    }

    /**
     * Sets the RPM of flywheel # 2
     * 
     * @param rpm RPM of the flywheel
     */
    public void setBottomMoterRPM(double rpm) {
        bottomTarget = rpm;
        shooterBottomMotor.setControl(bottomPID.withVelocity(rpm));
    }

    /**
     * @return The current RPM of flywheel # 1
     */
    public double getTopMotorRPM() {
        return (shooterTopMotor.getVelocity().getValue());
    }

    /**
     * @return The current RPM of flywheel # 2
     */
    public double getBottomMotorRPM() {
        return (shooterBottomMotor.getVelocity().getValue());
    }

    /**
     * @return Whether or not flywheel # 1 is on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean topMotorRPMOnTarget() {
        return Math.abs(getTopMotorRPM() - topTarget) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not flywheel # 2 is on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean bottomMotorRPMOnTarget() {
        return Math.abs(getBottomMotorRPM() - bottomTarget) < FlywheelConstants.RPM_TOLERANCE;
    }

    /**
     * @return Whether or not all flywheel motors are on target, within
     *         FlywheelConstants.RPM_TOLERANCE
     */
    public boolean allMotorsOnTarget() {
        return (topMotorRPMOnTarget() && bottomMotorRPMOnTarget());
    }

    public void coast() {
        shooterBottomMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
        shooterTopMotor.setVoltage(FlywheelConstants.COAST_VOLTAGE);
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
}
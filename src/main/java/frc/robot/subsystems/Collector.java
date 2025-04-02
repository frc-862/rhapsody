// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap.DIO;

public class Collector extends SubsystemBase {
    private ThunderBird motor;
    private DigitalInput beamBreak;
    private Debouncer debouncer = new Debouncer(0.2, DebounceType.kBoth);

    public Collector() {
        motor = new ThunderBird(CAN.COLLECTOR_MOTOR, CAN.CANBUS_FD, true, 0, false);
        beamBreak = new DigitalInput(DIO.COLLECTOR_BEAMBREAK);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Collector", "NoteState", noteCollected());
        LightningShuffleboard.setDouble("Collector", "collectorSpeed", motor.getVelocity().getValueAsDouble());
        LightningShuffleboard.set("Collector", "Command Scheduler", CommandScheduler.getInstance());
    }

    public void setSpeed(double speed){
        motor.setControl(new DutyCycleOut(speed));
    }

    public boolean noteCollected(){
        return debouncer.calculate(beamBreak.get());
    }

    public void stop(){
        setSpeed(0);
    }

}

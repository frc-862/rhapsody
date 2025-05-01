// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Indexer extends SubsystemBase {
    private ThunderBird motor;

    private DigitalInput entryBeam;
    private DigitalInput exitBeam;

    private Debouncer debouncer = new Debouncer(0.15, DebounceType.kBoth);

    public Indexer() {
        motor = new ThunderBird(CAN.INDEXER_MOTOR, CAN.CANBUS_FD, true, 0, false);

        entryBeam = new DigitalInput(RobotMap.DIO.INDEXER_ENTER_BEAMBREAK);
        exitBeam = new DigitalInput(RobotMap.DIO.INDEXER_EXIT_BEAMBREAK);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Indexer", "speed", motor.getVelocity().getValueAsDouble());

        LightningShuffleboard.setBool("Indexer", "entryBeam", getEntryBeam());
        LightningShuffleboard.setBool("Indexer", "exitBeam", getExitBeam());

        LightningShuffleboard.setBool("Indexer", "noteIndexed", isNoteIndexed());
    }

    public void setSpeed(double speed){
        motor.setControl(new DutyCycleOut(speed));
    }

    public boolean getEntryBeam(){
        return debouncer.calculate(entryBeam.get());
    }

    public boolean getExitBeam(){
        return exitBeam.get();
    }

    public boolean isNoteIndexed(){
        return getEntryBeam() && getExitBeam();
    }

}

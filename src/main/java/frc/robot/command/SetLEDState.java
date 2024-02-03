// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.subsystems.LEDs;

public class SetLEDState extends Command {
    LEDs leds;
    LED_STATES state;
    int length;
    int startTime;
    /** 
     * 
     * @param leds the LEDs subsystem
     * @param state the state to set the LEDs to
     * @param length the length of time to set the LEDs to the state (-1 for indefinite)
     */
    public SetLEDState(LEDs leds, LED_STATES state, int length) {
        this.leds = leds;
        this.state = state;
        this.length = length;
    }

    @Override
    public void initialize() {
        leds.setState(state, true);
        startTime = (int)Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        leds.setState(state, false);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime > length && length != -1); 
    }
}

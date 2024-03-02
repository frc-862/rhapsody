// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testcmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbTest extends Command {

    private Climber climber;
    private double power;

    /**
     * System test command for testing climb motors.
     * @param climber climber subsystem
     * @param power power to control up or down
     */
    public ClimbTest(Climber climber, double power) {
        this.climber = climber;
        this.power = power;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climber.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests.testcmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlywheelTest extends Command {

    private Flywheel flywheel;
    private double topMotorSpeed;
    private double bottomMotorSpeed;

    public FlywheelTest(Flywheel flywheel, double topMotorSpeed, double bottomMotorSpeed) {
        this.flywheel = flywheel;
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setTopMotorRPM(topMotorSpeed);
        flywheel.setBottomMotorRPM(bottomMotorSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.coast(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

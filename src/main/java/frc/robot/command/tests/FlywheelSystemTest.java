package frc.robot.command.tests;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testcmds.FlywheelTest;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class FlywheelSystemTest extends SystemTestCommandGroup {

    public FlywheelSystemTest(Flywheel flywheel, Collector collector, Indexer indexer, Pivot pivot, double speed) {
        super(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                // new InstantCommand(() -> { System.out.println("Start top"); }),
                new TimedCommand(new FlywheelTest(flywheel, speed, 0.0), 2), // Motor 1 out
                // new InstantCommand(() -> { System.out.println("Stop top"); }),
                new WaitCommand(1),
                new TimedCommand(new FlywheelTest(flywheel, -speed, 0.0), 2), // Motor 1 in
                new WaitCommand(1),
                new TimedCommand(new FlywheelTest(flywheel, 0.0, speed), 2), // Motor 2 out
                new WaitCommand(1),
                new TimedCommand(new FlywheelTest(flywheel, 0.0, -speed), 2), // Motor 2 in
                new WaitCommand(1),
                new TimedCommand(new FlywheelTest(flywheel, speed, speed), 2), // Both out
                new WaitCommand(1),
                new TimedCommand(new FlywheelTest(flywheel, -speed, -speed), 2) // Both in
            )  
        );
        addRequirements(flywheel);
    }
}

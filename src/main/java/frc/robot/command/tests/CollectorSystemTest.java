// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testcmds.CollectorTest;
import frc.robot.subsystems.Collector;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class CollectorSystemTest extends SystemTestCommandGroup {

    public CollectorSystemTest(Collector collector, Double power) {
        super(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new TimedCommand(new CollectorTest(collector,() -> power), 4), // Collector out
                new WaitCommand(1),
                new TimedCommand(new CollectorTest(collector,() -> -power), 4) // Collector in

            )
        );
        addRequirements(collector);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.SmartCollect;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class CycleSytemTest extends SystemTestCommandGroup {
  
    public CycleSytemTest(Collector collector, Indexer indexer, Pivot pivot, Flywheel flywheel, DoubleSupplier collectorPower, DoubleSupplier indexerPower, DoubleSupplier flywheelRPM) {
        super(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new TimedCommand(new SmartCollect(collectorPower, indexerPower, collector, indexer, pivot), 3), // Run smart collect for 3
                new WaitCommand(1),
                new TimedCommand(new ParallelCommandGroup( // Run two commands for 2 seconds
                    new StartEndCommand(() -> flywheel.setAllMotorsRPM(flywheelRPM.getAsDouble()), // Starts by setting flywheel RPM to flywheel RPM
                    () -> flywheel.coast(true)), // Slows down flywheel thru coasting
                    new RunCommand(() -> pivot.setTargetAngle(50)) // Meanwhile Moves pivot angle to 50 deg
                ), 2),
                new WaitCommand(0.5)
            )
        );
    }
}

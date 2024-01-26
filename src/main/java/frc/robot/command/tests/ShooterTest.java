// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.thunder.command.TimedCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterTest extends SequentialCommandGroup {
  
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;
  private Shooter shooter;
  private Collector collector;

  public ShooterTest(Shooter shooter, Flywheel flywheel, Collector collector, Indexer indexer, Pivot pivot) {
    this.shooter = shooter;
    this.flywheel = flywheel;
    this.collector = collector;
    this.indexer = indexer;
    this.pivot = pivot;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.5),
      new TimedCommand(new FlywheelTest(flywheel, 0.5, 0), 2), // Motor 1 out
      new WaitCommand(1),
      new TimedCommand(new FlywheelTest(flywheel, -0.5, 0), 2), // Motor 1 in
      new WaitCommand(1),
      new TimedCommand(new FlywheelTest(flywheel, 0, 0.5), 2), // Motor 2 out
      new WaitCommand(1),
      new TimedCommand(new FlywheelTest(flywheel, 0, -0.5), 2), // Motor 2 in
      new WaitCommand(1),
      new TimedCommand(new FlywheelTest(flywheel, 0.5, 0.5), 2), // Both out
      new WaitCommand(1),
      new TimedCommand(new FlywheelTest(flywheel, -0.5, -0.5), 2), // Both in
      new WaitCommand(1),
      new TimedCommand(new PivotTest(pivot, 0.5), 2), // Pivot up
      new WaitCommand(1),
      new TimedCommand(new PivotTest(pivot, -0.5), 2), // Pivot down
      new WaitCommand(1),
      new TimedCommand(new IndexerTest(indexer, 0.5), 2), // Indexer out
      new WaitCommand(1),
      new TimedCommand(new IndexerTest(indexer, -0.5), 2), // Indexer in
      new WaitCommand(1),
      new TimedCommand(new CollectorTest(collector, 0.5), 2), // Collector out
      new WaitCommand(1),
      new TimedCommand(new CollectorTest(collector, -0.5), 2) // Collector in
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.thunder.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
public class ClimbTest extends SequentialCommandGroup {
  
  private Climber climber;

  public ClimbTest(Climber climber) {
    this.climber = climber;
    addCommands(

      new WaitCommand(0.5),
      new TimedCommand(new ClimbMotorTest(climber, 1), 2), // climber out
      new WaitCommand(1),
      new TimedCommand(new ClimbMotorTest(climber, -1), 2), // climber 1 in

    )
  }
}

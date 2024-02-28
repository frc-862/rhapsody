// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.command.tests.testcmds.ClimbTest;
import frc.robot.subsystems.Climber;
import frc.thunder.command.TimedCommand;
import frc.thunder.testing.SystemTestCommandGroup;

public class ClimbSystemTest extends SystemTestCommandGroup {

    public ClimbSystemTest(Climber climber, double speed) {
        super(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new TimedCommand(new ClimbTest(climber, speed), 1), // UP
                new WaitCommand(1),
                new TimedCommand(new ClimbTest(climber, -speed), 1) // DOWN
            ) 
        );
    }
}

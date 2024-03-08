// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AmpAlign extends SequentialCommandGroup {
  /** Creates a new AmpAlign. */
  public AmpAlign(Pose2d target, Swerve drivetrain) {   
    // addCommands().
    //var roughMTP = new MoveToPose(target, drivetrain);
   // var  path = PathPlannerPath.fromPathFile("Example Path");
    //var followPath = AutoBuilder.followPath(path);
    super(new MoveToPose(target, drivetrain), buildPath());
    // addCommands(new FooCommand(), new BarCommand());
  }

  private static Command buildPath() {
    var  path = PathPlannerPath.fromPathFile("Amp autoalign");
    var followPath = AutoBuilder.followPath(path);
    // LightningShuffleboard.setBoolSupplier("AmpAlign", "path is running", () -> !followPath.isFinished());
    return followPath;
  }
}

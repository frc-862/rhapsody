// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;

public class AmpAlign extends SequentialCommandGroup {
  /** Creates a new AmpAlign. */
  public AmpAlign(Swerve drivetrain) {
    // sequentialy run move to pose to get near the amp followed by the path to align precisely
    super(buildMTP(drivetrain), buildPath());
  }

  /**
   * build Amp autoalign path
   * @return command to follow path
   */
  private static Command buildPath() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Amp autoalign");
    Command followPath = AutoBuilder.followPath(path);
    return followPath;
  }

  private static Command buildMTP(Swerve drivetrain){
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      return new MoveToPose(AutonomousConstants.AMP_POSE_RED_ALLIANCE, drivetrain);
    } else {
      return new MoveToPose(AutonomousConstants.AMP_POSE_BLUE_ALLIANCE, drivetrain);
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;



public class AlignToAmp extends Command {
  
  public Pose2d target;
  public Swerve drivetrain;
  public MoveToPose moveToPose;
  public PointAtTag pointAtTag;

  private PathPlannerPath path;

  /** Creates a new AlignToTag. */
  public AlignToAmp(Pose2d target, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path = PathPlannerPath.fromPathFile("Amp autoalign");
    moveToPose = new MoveToPose(target, drivetrain);
    moveToPose.initialize();
    new Trigger(() -> moveToPose.isFinished()).whileTrue(new AutoBuilder().buildAuto("Amp autoalign"));
    initLogging();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  public void initLogging() {
    LightningShuffleboard.setDoubleSupplier("AlignToAmp", "Target X", () -> target.getTranslation().getX());
    LightningShuffleboard.setDoubleSupplier("AlignToAmp", "Target Y", () -> target.getTranslation().getY());
    LightningShuffleboard.setBoolSupplier("AlignToAmp", "Aligning", () -> moveToPose.isFinished());
    LightningShuffleboard.setBoolSupplier("AlignToAmp", "isFinished", () -> isFinished());
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.hypot(drivetrain.getPose().getTranslation().getX() - target.getTranslation().getX(),
     drivetrain.getPose().getTranslation().getY() - target.getTranslation().getY()) 
     < AutonomousConstants.AMP_TOLERANCE;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.vision.Limelight;

public class UpdateOrientation extends Command {

  private Limelight stopMe;
  private Swerve drivetrain;
  private Pigeon2 pigeon;
  private double drivetrainYaw;
  private double drivetrainYawRate;
  
  /** Creates a new UpdateOrientation. */
  public UpdateOrientation(Limelights limelights, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stopMe = limelights.getStopMe();
    this.drivetrain = drivetrain;
    pigeon = drivetrain.getPigeon2();
    
    addRequirements(limelights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainYawRate = pigeon.getRate();
    if (!(drivetrainYawRate > 90)){
      drivetrainYaw = pigeon.getAngle();
      stopMe.setRobotOrientation((drivetrainYaw) % 360, drivetrainYawRate,55.5d,0,0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

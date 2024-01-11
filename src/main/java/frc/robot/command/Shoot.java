// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTargeting;

public class Shoot extends Command {
Shooter shooter = new Shooter();
ShooterTargeting shooterTargeting = new ShooterTargeting();

  public Shoot(Shooter shooter, ShooterTargeting shooterTargeting) {
    this.shooter = shooter;
    this.shooterTargeting = shooterTargeting;

    addRequirements(shooter, shooterTargeting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(shooterTargeting.getTargetFlywheelRPM());
    shooter.setAngle(shooterTargeting.getTargetFlywheelAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterTargeting.readyToFire());
    // TODO collect peice
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

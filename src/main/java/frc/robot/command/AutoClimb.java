// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class AutoClimb extends Command {
  /** Creates a new AutoCLimb. */
  private Pivot pivot;
  private Climber climber;
  private Swerve drivetrain;
  private boolean hasRetracted;

  public AutoClimb(Swerve drivetrain, Pivot pivot, Climber climber) {
    this.pivot = pivot;
    this.climber = climber;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setTargetAngle(pivot.getStowAngle());
    climber.deploy();
    hasRetracted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.isTipped()){
      climber.retract();
      hasRetracted = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!hasRetracted){
      climber.retract();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class ManualClimb extends Command {

  private Climber climber;
  private DoubleSupplier powerSupplier;

  /** Creates a new ManualClimb. */
  public ManualClimb(DoubleSupplier powerSupplier, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.powerSupplier = powerSupplier;
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPower(powerSupplier.getAsDouble());

    LightningShuffleboard.setDoubleSupplier("Climb", "Height", () -> climber.getHeight());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPower(powerSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
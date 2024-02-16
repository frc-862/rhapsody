package frc.robot.command.tests.testcmds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveTest extends Command {

  private Swerve drivetrain;
  private DoubleSupplier speedX;
  private DoubleSupplier speedY;

  /**
   * System test command for testing drive motors
   * @param drivetrain swerve subsystem
   * @param speedX X velocity
   * @param speedY Y velocity
   */
  public DriveTest(Swerve drivetrain, DoubleSupplier speedX, DoubleSupplier speedY) {
    this.drivetrain = drivetrain;
    this.speedX = speedX;
    this.speedY = speedY;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.applyRequestRobot(speedX, speedY, () -> 0d, 0d ,0d);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequestRobot(() -> 0d, () -> 0d, () -> 0d, 0d ,0d);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

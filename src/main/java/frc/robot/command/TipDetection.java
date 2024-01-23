package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TipDetection extends command {
  private Swerve drivetrain;
  public double pitch;
  public double roll;

  public TipDetection(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    // initiallizes pitch and roll (so we don't print null values)
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

    // displays whether the robot is off balance
    LightningShuffleboard.setBoolSupplier("TipDetection", "Tipped", () -> (Math.abs(pitch) > VisionConstants.COLLISION_DEADZONE || Math.abs(roll) > VisionConstants.COLLISION_DEADZONE));
  }

  @Override
  public void periodic() {
    // Updates roll and pitch values
    pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
    roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
  }
}

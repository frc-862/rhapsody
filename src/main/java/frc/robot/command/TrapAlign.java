// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.PathFindingConstants;
import frc.robot.Constants.ControllerConstants.ButtonBox;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TrapAlign extends Command {

    private Swerve drivetrain;
    private Joystick buttonBox = RobotContainer.buttonBox;
    private XboxControllerFilter driver = RobotContainer.driver;
    private Pose2d currentAlignPose;
    private Command pathCommand;

    private boolean pose1Active = false;
    private boolean pose2Active = false;
    private boolean pose3Active = false;

    /** Creates a new TrapAlign. */
    public TrapAlign(Swerve drivetrain) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      initLogging();
    }

    private void initLogging(){
      LightningShuffleboard.setBoolSupplier("TrapAlign", "FollowingPath", () -> pathCommand.isScheduled());
      LightningShuffleboard.setBoolSupplier("TrapAlign", "Pose 1 Active", () -> pose1Active);
      LightningShuffleboard.setBoolSupplier("TrapAlign", "Pose 2 Active", () -> pose2Active);
      LightningShuffleboard.setBoolSupplier("TrapAlign", "Pose 3 Active", () -> pose3Active);
    }

    private void updateButtons(){
      if (buttonBox.getRawButtonPressed(ButtonBox.PURPLE)){
        pose1Active = !pose1Active;
        pose2Active = false;
        pose3Active = false;
      } else if (buttonBox.getRawButtonPressed(ButtonBox.RED)){
        pose2Active = !pose2Active;
        pose1Active = false;
        pose3Active = false;
      } else if (buttonBox.getRawButtonPressed(ButtonBox.GRAY_BOTTOMRIGHT)){
        pose3Active = !pose3Active;
        pose1Active = false;
        pose2Active = false;
      }

      if (pose1Active){
        currentAlignPose = PathFindingConstants.TRAP_ALIGN_POSE1;
      } else if (pose2Active){
        currentAlignPose = PathFindingConstants.TRAP_ALIGN_POSE2;
      } else if (pose3Active){
        currentAlignPose = PathFindingConstants.TRAP_ALIGN_POSE3;
      }

      pathCommand = new PathToPose(currentAlignPose, drivetrain);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      updateButtons();

      if (driver.getYButton() && !pathCommand.isScheduled()){
        pathCommand.schedule();
      }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      pathCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}

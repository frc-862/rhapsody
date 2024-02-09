// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Sing extends Command {

  Orchestra sing;
  Swerve drivetrain;
  String filepath;

  /** Creates a new Sing. */
  /*
   * @param drivetrain used to grab the motors for the song
   * @param filepath used to select what song to sing
   * @param sing used to play the song and stop any current songs
   * please note that this removes all usage of the drivetrain until it is ended.
   * use with caution, and only when the drivetrain is not in use.
   */
  public Sing(Swerve drivetrain, String filepath, Orchestra sing) {
    this.drivetrain = drivetrain;
    this.filepath = filepath;
    this.sing = sing;
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    sing.clearInstruments();
    sing.stop();
    for (int i = 0; i < 4; i++){
      sing.addInstrument(drivetrain.getModule(i).getDriveMotor());
      sing.addInstrument(drivetrain.getModule(i).getSteerMotor());
    }
    sing.loadMusic(filepath);
    sing.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sing.stop();
    sing.clearInstruments();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

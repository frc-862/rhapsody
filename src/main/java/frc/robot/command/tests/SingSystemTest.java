// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.tests;

import com.ctre.phoenix6.Orchestra;

import frc.robot.subsystems.Swerve;
import frc.thunder.testing.SystemTestCommand;

public class SingSystemTest extends SystemTestCommand {
  
  private Orchestra sing = new Orchestra();

  private Swerve drivetrain;
  private String filepath;

  /*
   * please note that this removes all usage of the drivetrain until it is ended.
   * use with caution, and only when the drivetrain is not in use.
   * @param drivetrain used to grab the motors for the song
   * @param filepath used to select what song to sing
   */
  public SingSystemTest(Swerve drivetrain, String filepath) {
    this.drivetrain = drivetrain;
    this.filepath = filepath;
  }

  @Override
  public void initialize() {
    for (int i = 0; i < 4; i++){
      sing.addInstrument(drivetrain.getModule(i).getDriveMotor());
      sing.addInstrument(drivetrain.getModule(i).getSteerMotor());
    }
    sing.loadMusic(filepath);
    sing.play();
  }

  @Override
  public void execute() {
    if (!sing.isPlaying()){
      end(false);
      cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    sing.stop();
    sing.clearInstruments();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

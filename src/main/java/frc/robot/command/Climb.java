// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
import frc.robot.command.TipDetection;

public class Climb extends Command {
  /** Creates a new Climb. */

  // create vars
  private PIDController climbController = new PIDController(0.5, 0, 0); //TODO: tune pid
  private Climber climber;
  private double setPoint;
  private TipDetection tipDetection;

  public Climb(Climber climber, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    // initialize variables
    this.climber = climber;
    this.tipDetection = new TipDetection(drivetrain);

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = ClimbConstants.CLIMB_PID_SETPOINT_EXTENDED;
  }

  /**
   * retracts climb
   */
  public void retract(){
    setPoint = ClimbConstants.CLIMB_PID_SETPOINT_RETRACTED;
  }

  /**
   * @return returns whether climb is extended
   */
  public boolean extended(){
    return (setPoint - climber.getHeight() < ClimbConstants.CLIMB_EXTENSION_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // use pid to set climber power
    double pidOutput = climbController.calculate(climber.getHeight(), setPoint);
    climber.setPower(pidOutput);

    if(extended() && tipDetection.isTipped()){
      retract();
    }
  }


  /**
   * @return climb setpoint
   */
  public double getSetPoint(){
    return setPoint;
  }

  /**
   * set setpoint for climb pid
   */
  public void setSetPoint(double setPoint){
    this.setPoint = setPoint;
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

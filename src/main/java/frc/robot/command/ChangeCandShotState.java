// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.CAND_STATES;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;
import frc.robot.subsystems.Shooter;

public class ChangeCandShotState extends Command {
    private Shooter shooter;
	private BooleanSupplier copilotAButton;
	private BooleanSupplier copilotBButton;
	private BooleanSupplier copilotXButton;
    /** Creates a new ChangeCandShotState. */
    public ChangeCandShotState() {
		this.shooter = shooter;
		this.copilotAButton = copilotAButton;
		this.copilotBButton = copilotBButton;
		this.copilotXButton = copilotXButton;
		addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
		// sets shooter state to cand shots
		shooter.setState(SHOOTER_STATES.CAND_SHOTS);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
		// Sets cand states based off which button is pressed
		if(copilotAButton.getAsBoolean()){
			shooter.setCANDState(CAND_STATES.AMP);	
		} else if (copilotBButton.getAsBoolean()){
			shooter.setCANDState(CAND_STATES.SUBWOOFER);	
		} else {
			shooter.setCANDState(CAND_STATES.PODIUM);	
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

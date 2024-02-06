// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class AmpShot extends Command {
	/** Creates a new CANDShots. */

	private Pivot pivot;
	private Flywheel flywheel;

	public AmpShot(Pivot pivot, Flywheel flywheel) {
		this.flywheel = flywheel;
		this.pivot = pivot;
	
		addRequirements(pivot);
		addRequirements(flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(ShooterConstants.AMP_RPM);
		pivot.setTargetAngle(ShooterConstants.AMP_ANGLE);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		flywheel.setAllMotorsRPM(ShooterConstants.STOW_RPM);
		pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

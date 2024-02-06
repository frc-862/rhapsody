// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class Stow extends Command {
	/** Creates a new Stow. */

	private Pivot pivot;
	private Flywheel flywheel;

	public Stow(Pivot pivot, Flywheel flywheel) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(pivot);
		addRequirements(flywheel);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		flywheel.coast();
		pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		//TODO add LED state
	}
}
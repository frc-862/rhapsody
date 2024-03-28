// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.thunder.command.TimedCommand;

public class NotePass extends Command {
  private final Flywheel flywheel;
	private final Pivot pivot;

	/**
	 * Creates a new NotePass.
	 *
	 * @param pivot    subsystem
	 * @param flywheel subsystem
	 */
	public NotePass(Flywheel flywheel, Pivot pivot) {
		this.flywheel = flywheel;
		this.pivot = pivot;

		addRequirements(flywheel, pivot);
	}

	@Override
	public void initialize() {
		flywheel.setAllMotorsRPM(CandConstants.NOTE_PASS_RPM + flywheel.getBias());
		pivot.setTargetAngle(CandConstants.NOTE_PASS_ANGLE + pivot.getBias());
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(CandConstants.NOTE_PASS_RPM + pivot.getBias());
		pivot.setTargetAngle(CandConstants.NOTE_PASS_ANGLE + flywheel.getBias());
		if(flywheel.allMotorsOnTarget() && pivot.onTarget()) {
			new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
		}
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

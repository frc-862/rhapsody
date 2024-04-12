// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class NotePass extends Command {

	private final Swerve drivetrain;
  	private final Flywheel flywheel;
	private final Pivot pivot;

	private BooleanSupplier doHaptic;

	/**
	 * Creates a new NotePass.
	 *
	 * @param drivetrain subsystem
	 * @param pivot    subsystem
	 * @param flywheel subsystem
	 * @param doHaptic used for checking point on point on target
	 */
	public NotePass(Swerve drivetrain, Flywheel flywheel, Pivot pivot, BooleanSupplier doHaptic) {
		this.drivetrain = drivetrain;
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.doHaptic = doHaptic;

		addRequirements(flywheel, pivot);
	}

	public NotePass(Swerve drivetrain, Flywheel flywheel, Pivot pivot) {
		this(drivetrain, flywheel, pivot, () -> true);
	}

	@Override
	public void initialize() {

		LightningShuffleboard.setDoubleSupplier("NotePass", "Distance to Corner", () -> drivetrain.distanceToCorner());
	}

	@Override
	public void execute() {
		flywheel.setAllMotorsRPM(ShooterConstants.NOTEPASS_SPEED_MAP.get(drivetrain.distanceToCorner()) + flywheel.getBias());
		pivot.setTargetAngle(ShooterConstants.NOTEPASS_ANGLE_MAP.get(drivetrain.distanceToCorner()) + pivot.getBias());

		if (flywheel.allMotorsOnTarget() && pivot.onTarget() && doHaptic.getAsBoolean())
			new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
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

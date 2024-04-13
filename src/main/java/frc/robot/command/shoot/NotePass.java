// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.thunder.command.TimedCommand;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class NotePass extends Command {

	private final Swerve drivetrain;
  	private final Flywheel flywheel;
	private final Pivot pivot;
	private Translation2d targetPose;
	private double currentHeading;
	private double targetHeading;
	private double feedForwardOutput;
	private double pidOutput;
	private PIDController pidController = VisionConstants.COMBO_CONTROLLER; // TAG_AIM_CONTROLLER
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25, 0.5);
	private XboxControllerFilter driver;
	private Indexer indexer;

	/**
	 * Creates a new NotePass.
	 *
	 * @param drivetrain subsystem
	 * @param pivot    subsystem
	 * @param flywheel subsystem
	 * @param driver   the driver's controller, used for drive input
	 * @param indexer  subsystem
	 */
	public NotePass(Swerve drivetrain, Flywheel flywheel, Pivot pivot, XboxControllerFilter driver, Indexer indexer) {
		this.drivetrain = drivetrain;
		this.flywheel = flywheel;
		this.pivot = pivot;
		this.driver = driver;
		this.indexer = indexer;

		addRequirements(flywheel, pivot, drivetrain, indexer);
	}

	@Override
	public void initialize() {
		if(isBlueAlliance()){
			targetPose = DrivetrainConstants.BLUE_CORNER_POSE;
		} else {
			targetPose = DrivetrainConstants.RED_CORNER_POSE;
		}
	}

	@Override
	public void execute() {
		Pose2d pose = drivetrain.getPose();
        var deltaX = targetPose.getX() - pose.getX();
        var deltaY = targetPose.getY() - pose.getY();

        currentHeading = (pose.getRotation().getDegrees() + 360) % 360;

        // Calculate vector to target, add 180 to make it point backwards
        targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180; 
        targetHeading = (targetHeading + 360) % 360; // Modulo 360 to keep it in the range of 0-360
        
        pidOutput = pidController.calculate(currentHeading, targetHeading);
        feedForwardOutput = feedforward.calculate(pidOutput);
        
        if(inTolerance()) {
            feedForwardOutput = 0;
        }

        drivetrain.setField(-driver.getLeftY(), -driver.getLeftX(), feedForwardOutput);
		
		flywheel.setAllMotorsRPM(ShooterConstants.NOTEPASS_SPEED_MAP.get(drivetrain.distanceToCorner()) + flywheel.getBias());
		pivot.setTargetAngle(ShooterConstants.NOTEPASS_ANGLE_MAP.get(drivetrain.distanceToCorner()) + pivot.getBias());

		if (flywheel.allMotorsOnTarget() && pivot.onTarget() /*&& inTolerance() */) {
			indexer.indexUp();
			new TimedCommand(RobotContainer.hapticCopilotCommand(), 1d).schedule();
			new TimedCommand(RobotContainer.hapticDriverCommand(), 1d).schedule();
		}

		if (!DriverStation.isFMSAttached()) {
			LightningShuffleboard.setBool("Note-Pass", "In tloeracnce", inTolerance());
			LightningShuffleboard.setDouble("Note-Pass", "CurrentHeading", currentHeading);
			LightningShuffleboard.setDouble("Note-Pass", "TargetHeading", targetHeading);
			LightningShuffleboard.setDouble("Note-Pass", "Distance to Corner", drivetrain.distanceToCorner());
		}
	}

	@Override
	public void end(boolean interrupted) {
		flywheel.coast(true);
		pivot.setTargetAngle(pivot.getStowAngle());
		indexer.stop();
	}

	@Override
	public boolean isFinished() {
		return !indexer.hasNote();
	}

	private boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

	private boolean inTolerance() {
        double difference = Math.abs(currentHeading - targetHeading);
        difference = difference > 180 ? 360 - difference : difference;
        return difference <= VisionConstants.POINTATPOINT_ALIGNMENT_TOLERANCE; // TODO not has Target but if the correct filter is set
    }
}

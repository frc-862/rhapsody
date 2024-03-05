// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class PointAtPointAuton extends Command {

    public Swerve drivetrain;

    private double pidOutput;
	private double targetHeading;
	private Translation2d targetPose;

    private PIDController headingController = VisionConstants.TAG_AIM_CONTROLLER;

    /**
	 * Creates a new PointAtTagAuton.
	 * @param drivetrain to request movement
	 */
    public PointAtPointAuton(Swerve drivetrain) {
        this.drivetrain = drivetrain;
		    this.targetPose = new Translation2d();
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose().get();
		var deltaX = targetPose.getX() - pose.getX();
		var deltaY = targetPose.getY() - pose.getY();

		targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
		targetHeading += 180;
		pidOutput = headingController.calculate(pose.getRotation().getDegrees(), targetHeading);

        drivetrain.setField(0, 0, pidOutput);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

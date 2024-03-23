// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.filter.XboxControllerFilter;

public class AutonPointAtTag extends PointAtTag {

    public AutonPointAtTag(Swerve swerve, Limelights limelights, XboxControllerFilter driver) {
        super(swerve, limelights, driver);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(limelights.getStopMe().getTargetX()) < VisionConstants.POINTATTAG_ALIGNMENT_TOLERANCE) && limelights.getStopMePipeline() == VisionConstants.SPEAKER_POINT_PIPELINE;
    }
}

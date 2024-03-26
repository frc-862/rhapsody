// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CandConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Pivot;

public class ReverseAmpShot extends Command {
    
    private final Flywheel flywheel;
    private final Pivot pivot;

    public ReverseAmpShot(Flywheel flywheel, Pivot pivot) {
        this.flywheel = flywheel;
        this.pivot = pivot;

        addRequirements(flywheel, pivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pivot.setTargetAngle(CandConstants.REVERSE_AMP_ANGLE + pivot.getBias());
        flywheel.setTopMotorRPM(CandConstants.REVERSE_AMP_TOP_RPM + flywheel.getBias());
        flywheel.setBottomMotorRPM(CandConstants.REVERSE_AMP_BOTTOM_RPM + flywheel.getBias());
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setTargetAngle(pivot.getStowAngle());
        flywheel.coast(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

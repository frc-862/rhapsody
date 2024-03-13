// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToPose extends Command {

    private final Pose2d target;
    private final Swerve drivetrain;

    private boolean finished = false;
    private Pose2d current;
    private double dx;
    private double dy;
    private final double kp = 1.8;
    private final double minSpeed = 0.9;
    private double powerx;
    private double powery;

    private DoubleLogEntry dxLog;
    private DoubleLogEntry dyLog;
    private DoubleLogEntry targetXLog;
    private DoubleLogEntry targetYLog;

    /**
     * @param target     The target pose to move to
     * @param drivetrain The drivetrain subsystem
     */
    public MoveToPose(Pose2d target, Swerve drivetrain) {
        this.target = target;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

        initLogging();
    }

    @Override
    public void initialize() {}

    /**
     * initialize logging
     */
    private void initLogging() {
        DataLog log = DataLogManager.getLog();

        dxLog = new DoubleLogEntry(log, "/MoveToPose/dx");
        dyLog = new DoubleLogEntry(log, "/MoveToPose/dy");
        targetXLog = new DoubleLogEntry(log, "/MoveToPose/targetX");
        targetYLog = new DoubleLogEntry(log, "/MoveToPose/targetY");
    }

    @Override
    public void execute() {
        current = drivetrain.getPose();
        dx = target.getTranslation().getX() - current.getTranslation().getX();
        dy = target.getTranslation().getY() - current.getTranslation().getY();

        powerx = dx * kp;
        powery = dy * kp;

        if (minSpeed > Math.abs(powerx)) {
            powerx = minSpeed * Math.signum(dx);
        }

        if (minSpeed > Math.abs(powery)) {
            powery = minSpeed * Math.signum(dy);
        }

        if (dx == 0) {
            powerx = 0;
        }

        if (dy == 0) {
            powery = 0;
        }

        var dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 0.3) {
            powerx = 0;
            powery = 0;
            finished = true;
        }

        drivetrain.setField(powerx, powery, 0); // TODO test

        updateLogging();
    }

    /**
     * update logging
     */
    public void updateLogging() {
        dxLog.append(dx);
        dyLog.append(dy);
        targetXLog.append(target.getTranslation().getX());
        targetYLog.append(target.getTranslation().getY());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.brake(); // TODO test and decide if this is the desired behavior
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

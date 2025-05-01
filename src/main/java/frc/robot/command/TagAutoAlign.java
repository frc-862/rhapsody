// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;

import javax.naming.AuthenticationNotSupportedException;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.PivotRhapsody;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.vision.Limelight;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;;

public class TagAutoAlign extends Command {
    private Limelights limelights;
    private PivotRhapsody pivot;
    private Swerve drivetrain;

    private PIDController rPid;
    private PIDController pivotPid;

    private DoubleSupplier leftx;
    private DoubleSupplier lefty;
    private double tx;

    private double rTolerance;
    private double pivotTolerance;

    private double rSpeed;
    private Debouncer debouncer = new Debouncer(0.1, DebounceType.kBoth);

    public TagAutoAlign(Limelights limelights, PivotRhapsody pivot, Swerve drivetrain, DoubleSupplier leftx, DoubleSupplier lefty){
        this.limelights = limelights;
        this.pivot = pivot;
        this.drivetrain = drivetrain;

        this.leftx = leftx;
        this.lefty = lefty;

        addRequirements(limelights, pivot, drivetrain);
    }

    @Override
    public void initialize() {
        rPid = new PIDController(AutonomousConstants.TAGALIGN_RP, AutonomousConstants.TAGALIGN_RI, AutonomousConstants.TAGALIGN_RD);
        rPid.setTolerance(AutonomousConstants.RTOLERANCE);
        rPid.setSetpoint(0);
    }

    @Override
    public void execute() {

        tx = limelights.getShooterLimeLight().getTargetX();

        rSpeed = rPid.calculate(tx);
        LightningShuffleboard.setBool("TagAutoAlign", "rPid setpoint", debouncer.calculate(rPid.atSetpoint()));

        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(MathUtil.applyDeadband(lefty.getAsDouble() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND))
            .withVelocityY(MathUtil.applyDeadband(leftx.getAsDouble() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND))
            .withRotationalRate(rSpeed));
    }
    
    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(MathUtil.applyDeadband(lefty.getAsDouble() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND))
            .withVelocityY(MathUtil.applyDeadband(leftx.getAsDouble() * drivetrain.getSpeedMult(), ControllerConstants.DEADBAND))
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(rPid.atSetpoint());
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class MoveToPose extends Command {
    private final Pose2d target; 
    private final Swerve drivetrain;
    private FieldCentric drive; 
    private boolean finished;
    private Pose2d current;
    private double dx;
    private double dy;
    private final double kp = 0.8; //1.8 old
    private final double minSpeed = 0.4; //0.9 old
    private double powerx;
    private double powery;
    /** 
     * @param target The target pose to move to
     * @param drivetrain The drivetrain subsystem
     * @param drive The drive mode
    */
    public MoveToPose(Pose2d target, Swerve drivetrain, SwerveRequest.FieldCentric drive) {
        this.drive = drive;
        this.target = target; 
        this.drivetrain = drivetrain;

        addRequirements(drivetrain); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("I'm working heheheha!!!!!################################333333!");
        LightningShuffleboard.setDouble("MoveToPose", "dx", dx);
        LightningShuffleboard.setDouble("MoveToPose", "dy", dy);
        LightningShuffleboard.setDouble("MoveToPose", "targetX", target.getX());
        LightningShuffleboard.setDouble("MoveToPose", "targetY", target.getY());

        current = drivetrain.getPose().get(); 
        System.out.println(current);
        dx = target.getTranslation().getX() - current.getTranslation().getX();
        dy = target.getTranslation().getY() - current.getTranslation().getY();
        System.out.println(dx);
        System.out.println(dy);
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

        var dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < 0.4) {
            powerx = 0;
            powery = 0;
            finished = true;
        }
        drivetrain.setControl(drive.withVelocityX(powerx).withVelocityY(powery));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}

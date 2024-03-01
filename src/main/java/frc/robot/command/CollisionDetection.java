// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollisionConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.CollisionConstants.CollisionType;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class CollisionDetection extends Command {

    private Swerve drivetrain;

    public Rotation2d collisionDirection;

    public double[] rotVelP = {0d, 0d};
    public double[] time = {0d, 0d};
    public double[] xVelC = {0d, 0d};
    public double[] yVelC = {0d, 0d};
    public double[] rotVelC = {0d, 0d};

    // filters for filtering acceleration values
    public LinearFilter xAccCFilter = LinearFilter.singlePoleIIR(0.1, 0.01);
    public LinearFilter yAccCFilter = LinearFilter.singlePoleIIR(0.1, 0.01);
    public LinearFilter rotAccCFilter = LinearFilter.singlePoleIIR(0.1, 0.01);

    double accelerationTolerance;
    double minAccelerationDiff;
    CollisionType type;

    public CollisionDetection(Swerve drivetrain, CollisionType type) {
        this.drivetrain = drivetrain;
        this.type = type;
    }

    @Override
    public void initialize() {
        storeVelocities(); // store initial velocities to avoid null values
        setDisplayAccelerationTolerances(type); // set acceleration tolerance based on type
    }

    @Override
    public void execute() {
        storeVelocities();
        // log values to shuffleboard
        LightningShuffleboard.setDouble("Collision Detection", "pidgeon acceleration magnitude", getPigeonAcceleration()[2]);
        LightningShuffleboard.setDouble("Collision Detection", "pigeon accelaration direction", getPigeonAcceleration()[3]);
        LightningShuffleboard.setDouble("Collision Detection", "pigeon anglular acceleration", getPigeonAcceleration()[4]);
        LightningShuffleboard.setDouble("Collision Detection", "motor acceleration magnitude", getChassisAcceleration()[2]);
        LightningShuffleboard.setDouble("Collision Detection", "motor angular acceleration", getChassisAcceleration()[4]);
        LightningShuffleboard.setBool("Collision Detection", "collided", getIfCollided()[3]);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * log velocities to calculate acceleration
     */
    public void storeVelocities() {
        rotVelP[0] = rotVelP[1];
        time[0] = time[1];
        xVelC[0] = xVelC[1];
        yVelC[0] = yVelC[1];
        rotVelC[0] = rotVelC[1];

        // store velocities to calculate acceleration
        rotVelP[1] = Units.degreesToRadians(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        time[1] = Utils.getCurrentTimeSeconds();
        xVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond / TunerConstants.kDriveGearRatio;
        yVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond / TunerConstants.kDriveGearRatio;
        rotVelC[1] = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond / TunerConstants.kDriveGearRatio;
    }

    /**
     * @return array with pigeon acceleration
     * <li> 0 - Acceleration in X direction
     * <li> 1 - Acceleration in Y direction
     * <li> 2 - Magnitude of X and Y Acceleration
     * <li> 3 - Direction of X and Y Acceleration
     * <li> 4 - Rotational Acceleration
     */
    public double[] getPigeonAcceleration() {
        double accX = drivetrain.getPigeon2().getAccelerationX().getValueAsDouble()
            - drivetrain.getPigeon2().getGravityVectorX().getValueAsDouble() // subtract gravity from acceleration
            * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

        double accY = drivetrain.getPigeon2().getAccelerationY().getValueAsDouble()
            - drivetrain.getPigeon2().getGravityVectorY().getValueAsDouble() // subtract gravity from acceleration
            * CollisionConstants.ACCELERATION_DUE_TO_GRAVITY; // convert g-force to m/s^2

        double accMag = Math.hypot(accX, accY); // calculate magnitude of acceleration
        double accDirection = Math.atan2(accY, accX); // calculate direction of acceleration
        double accRot = (rotVelP[1] - rotVelP[0]) / (time[1] - time[0]); // calculate rotational acceleration

        return new double[] {accX, accY, accMag, accDirection, accRot};
    }

    /**
     * @return array with chassis acceleration
     * <li> 0 - Acceleration in X direction
     * <li> 1 - Acceleration in Y direction
     * <li> 2 - Magnitude of X and Y Acceleration
     * <li> 3 - Direction of X and Y Acceleration
     * <li> 4 - Rotational Acceleration
     * <li> 5 - Raw X Acceleration (without rotational acceleration)
     * <li> 6 - Raw Y Acceleration (without rotational acceleration)
     * <li> 7 - Raw Direction of Acceleration (without rotational acceleration)
     */
    public double[] getChassisAcceleration() {
        double deltaTime = time[1] - time[0];
        double accX = xAccCFilter.calculate((xVelC[1] - xVelC[0]) / deltaTime); // calculate acceleration in x direction and filter
        double accY = yAccCFilter.calculate((yVelC[1] - yVelC[0]) / deltaTime); // calculate acceleration in y direction and filter
        double accRot = rotAccCFilter.calculate((rotVelC[1] - rotVelC[0]) / deltaTime); // calculate rot acceleration and filter
        double accDirection = Math.atan2(accY, accX); // calculate direction of acceleration
        double accMag = Math.hypot(accX, accY); // calculate magnitude of acceleration

        return new double[] {accX, accY, accMag, accDirection, accRot};
    }

    /**
     * <li> sets main acceleration tolerance and minimum acceleration difference
     * <li> the getIfCollided method can be used seperately with different values
     * @param collisionType - enum in constants
     */
    public void setDisplayAccelerationTolerances(CollisionType collisionType) {
        switch (collisionType) {
            case AUTON:
                accelerationTolerance = CollisionConstants.ACCELERATION_TOLERANCE_AUTON;
                minAccelerationDiff = CollisionConstants.MIN_ACCELERATION_DIFF_AUTON;
                break;
            case SHOOTER:
                accelerationTolerance = CollisionConstants.ACCELERATION_TOLERANCE_SHOOTER;
                minAccelerationDiff = CollisionConstants.MIN_ACCELERATION_DIFF_SHOOTER;
                break;
            case TELEOP:
                accelerationTolerance = CollisionConstants.ACCELERATION_TOLERANCE_TELEOP;
                minAccelerationDiff = CollisionConstants.MIN_ACCELERATION_DIFF_TELEOP;
                break;
        }
    }

    /**
     * Compare Pigeon and Chassis Acceleraiton
     * @param accelerationTolerance percentage of pigeon acceleration to compare to
     * @param minAccelerationDiff if the difference in acceleration is less than this value, it in not considered a collision
     * @return boolean array
     * <li> 0 - check x
     * <li> 1 - check y
     * <li> 2 - check rot
     * <li> 3 - check all
     */
    public boolean[] getCollided(double accelerationTolerance, double minAccelerationDiff) {
        double differenceX = Math.abs(getPigeonAcceleration()[0] - getChassisAcceleration()[0]); // calculate difference in x acceleration
        boolean xCollided = differenceX > getPigeonAcceleration()[0] * accelerationTolerance
            && differenceX > minAccelerationDiff; // check if difference is greater than tolerance and min difference

        double differenceY = Math.abs(getPigeonAcceleration()[1] - getChassisAcceleration()[1]);
        boolean yCollided = differenceY > getPigeonAcceleration()[1] * accelerationTolerance
            && differenceY > minAccelerationDiff;

        double differenceRot = Math.abs(getPigeonAcceleration()[4] - getChassisAcceleration()[4]);
        boolean rotCollided = differenceRot > getPigeonAcceleration()[4] * accelerationTolerance
            && differenceX > minAccelerationDiff;

        collisionDirection = new Rotation2d(Math.atan2(differenceY, differenceX));

        boolean collided = xCollided || yCollided || rotCollided;

        return new boolean[] {xCollided, yCollided, rotCollided, collided};
    }

    /**
     * uses state from when command was called for tolerances
     * @return boolean array
     * <li> 0 - check x
     * <li> 1 - check y
     * <li> 2 - check rot
     * <li> 3 - check all
     */
    public boolean[] getIfCollided() {
        return getCollided(accelerationTolerance, minAccelerationDiff);
    }
}
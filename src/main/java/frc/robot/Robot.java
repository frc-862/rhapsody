package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.thunder.LightningRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LightningRobot {

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    protected void allianceKnown(Alliance alliance) {
        RobotContainer container = (RobotContainer) getContainer();
        if (alliance == Alliance.Red) {
            container.drivetrain
                    .setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(180)));
        } else {
            container.drivetrain
                    .setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(0)));
        }
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();

        RobotContainer container = (RobotContainer) getContainer();
        if (haveDriverStation) {
            container.drivetrain.setAlliancePose(DriverStation.getAlliance().get());
        }
        container.pivot.resetBias();

        container.drivetrain.brake();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();

        RobotContainer container = (RobotContainer) getContainer();
        container.drivetrain.disableVision();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();

        RobotContainer container = (RobotContainer) getContainer();
        container.drivetrain.enableVision();
    }
}

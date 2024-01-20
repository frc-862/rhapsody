package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;

public class Shooter extends SubsystemBase {
    private Pivot pivot;
    private Flywheel flywheel;
    private XboxController copilot;

    //TODO: add collector/indexer

    //TODO: find initial target angle
    private double currentDistance = 0;

    private SHOOTER_STATES state = SHOOTER_STATES.STOW;

    private Swerve drivetrain;

    public Shooter(Pivot pivot, Flywheel flywheel, Swerve drivetrain, XboxController copilot) {
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        this.copilot = copilot;
    }
  
    @Override
    public void periodic() {
        var xPose = drivetrain.getPose().get().getX(); 
        var yPose = drivetrain.getPose().get().getX(); 

        if (yPose > ShooterConstants.CLOSE_WING_Y && yPose < ShooterConstants.FAR_WING_Y && 
        xPose > ShooterConstants.CLOSE_WING_X && xPose < ShooterConstants.FAR_WING_X) {
            if (copilot.getAButton()) { 
                state = SHOOTER_STATES.SHOOT;
            } else {
                state = SHOOTER_STATES.PRIME;
            }
        } else {
            state = SHOOTER_STATES.STOW;
        }


        //TODO: IMPLEMENT VISION/ODO
        currentDistance = 0;
        switch (state) {
            //Stow pivot to allow collector to input not\
            case STOW:
                pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
                flywheel.setRPM(0);
                break;
            //Prime is warming up the flywheel for shooting
            case PRIME:
                pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
                flywheel.setRPM(ShooterConstants.BASE_RPM);
                break;
            //Aim is targeting the pivot toward the target and speeding up the flywheel
            case AIM:
                pivot.setTargetAngle(ShooterConstants.ANGLE_MAP.get(currentDistance));
                flywheel.setRPM(ShooterConstants.SPEED_MAP.get(currentDistance));
                break;
            // Shoot is shooting the note
            case SHOOT:
                pivot.setTargetAngle(ShooterConstants.ANGLE_MAP.get(currentDistance));         
                flywheel.setRPM(ShooterConstants.SPEED_MAP.get(currentDistance));

                //If flywheel and pivot are on target (ready to shoot), shoot               
                if(flywheel.onTarget() && pivot.onTarget()) {
                    //index
                }
                break;
        }
    }

    public SHOOTER_STATES getState() {
        return state;

}

    /**
     * Sets the state of the shooter (STOW, PRIME, AIM, SHOOT)
     * note: 
     * @param state
     */
    public void setState(SHOOTER_STATES state) {
        this.state = state;
    }
}

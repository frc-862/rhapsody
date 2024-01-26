package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;

public class Shooter extends SubsystemBase {
    private Pivot pivot;
    private Flywheel flywheel;
    private Indexer indexer;
    private Collector collector;

    private SHOOTER_STATES state = SHOOTER_STATES.STOW;
    private double currentDistance = 0;
    private boolean shoot = false;

    /**
     * Shooter Subsystem
     * @param pivot Sets target angle
     * @param flywheel Sets target RPM
     * @param indexer For peice beambrake
     * @param collector For peice beambrakes
     */
    public Shooter(Pivot pivot, Flywheel flywheel, Indexer indexer, Collector collector) {
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.collector = collector;
    }
  
    @Override
    public void periodic() {
        //TODO: IMPLEMENT VISION/ODO
        switch (state) {
            //Stow pivot to allow collector to input not\
            case STOW:
                pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
                flywheel.setAllMotorsRPM(0);
                break;
            //Prime is warming up the flywheel for shooting
            case PRIME:
                pivot.setTargetAngle(ShooterConstants.STOW_ANGLE);
                flywheel.setAllMotorsRPM(ShooterConstants.BASE_RPM);
                break;
            //Aim is targeting the pivot toward the target and speeding up the flywheel
            case AIM:
                pivot.setTargetAngle(ShooterConstants.ANGLE_MAP.get(currentDistance));
                flywheel.setAllMotorsRPM(ShooterConstants.SPEED_MAP.get(currentDistance));
                break;
            // Shoot is shooting the note
            case SHOOT:
                pivot.setTargetAngle(ShooterConstants.ANGLE_MAP.get(currentDistance));         
                flywheel.setAllMotorsRPM(ShooterConstants.SPEED_MAP.get(currentDistance));

                //If flywheel and pivot are on target (ready to shoot), shoot               
                if(flywheel.allMotorsOnTarget() && pivot.onTarget()) {
                    indexer.indexIn();
                }
                break;
        }
    }

    /**
     * Get current shooter state
     * @return SHOOTER_STATES
     */
    public SHOOTER_STATES getState() {
        return state;
    }

    /**
     * Sets the state of the shooter (STOW, PRIME, AIM, SHOOT)
     * @param state
     */
    public void setState(SHOOTER_STATES state) {
        this.state = state;
    }

    /**
     * Distance from target, no current logic
     * @return distance to target in meters
     */
    public double getDistanceToTarget() {
        return currentDistance; // TODO FRITZ add on the fly here or we can add more logic
    }

    /**
     * Get shoot
     * @return boolean if the shot has been requested
     */
    public boolean getShoot() {
        return shoot;
    }
}
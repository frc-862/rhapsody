package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SHOOTER_STATES;
import frc.robot.Constants.ShooterConstants.CAND_STATES;

public class Shooter extends SubsystemBase {
    private Pivot pivot;
    private Flywheel flywheel;
    private Indexer indexer;
    private Collector collector;

    private SHOOTER_STATES state = SHOOTER_STATES.STOW;
    private CAND_STATES candState = CAND_STATES.AMP;
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
            
            //set shots for the pivot and flywheel
            case CAND_SHOTS:
                pivot.setTargetAngle(ShooterConstants.CAND_SHOT_VALUES[candState.getPriority()][0]);
                flywheel.setAllMotorsRPM(ShooterConstants.CAND_SHOT_VALUES[candState.getPriority()][1]);
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
     * get current cand shot state
     * @param candState
     */
    public CAND_STATES getCANDState() {
        return candState;
    }

    /**
     * Sets the states for cand shots (AMP, SUBWOOFER, PODIUM)
     * @param candState
     */
    public void setCANDState(CAND_STATES candState) {
        this.candState = candState;
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.thunder.hardware.ThunderBird;
import frc.robot.Constants.RobotMap.CAN;

public class Climber extends SubsystemBase {
    
    private ThunderBird leftClimb;
    private ThunderBird rightClimb;

    public Climber() {
        leftClimb = new ThunderBird(CAN.CLIMB_LEFT, CAN.CANBUS_FD , false, 0, false);
        rightClimb = new ThunderBird(CAN.CLIMB_RIGHT, getName(), false, 0, false);
    }

    @Override
    public void periodic() {
        
    }
}

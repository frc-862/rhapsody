// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public class drivetrainConstants {
        
    }

    public class RobotMap {
        public class CAN {
            public static final int SHOOTER_MOTOR_1 = 0;
            public static final int SHOOTER_MOTOR_2 = 0;

            public static final String CANBUS = "Canivore";
        }
    }

    public class VisionConstants  {
         //This is a magic number from gridlock, may need to be changed or removed entirely
            public static final double PROCESS_LATENCY = 0.0472; // TODO test
            public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
            public static final Translation2d VISION_LIMIT = new Translation2d(Units.feetToMeters(9), Units.feetToMeters(5));
    }
    
    public class ShooterConstants{
        public static final double SHOOTER_SPEED = 0.5; // magic number :D
        public static final boolean MOTOR_1_INVERT = false;
        public static final boolean MOTOR_2_INVERT = false;
        public static final int SUPPLY_CURRENT_LIMIT = 0;
        public static final int STATOR_CURRENT_LIMIT = 0;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
    }
    public class ControllerConstants{
        public static final int COPILOT_CONTROLLER_PORT = 0;
    }
}
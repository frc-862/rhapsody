// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public class drivetrainConstants {
        
    }

    public class RobotMap {
        public class CAN {

        }
    }

    public class VisionConstants  {
         //This is a magic number from gridlock, may need to be changed or removed entirely
            public static final double PROCESS_LATENCY = 0.0472; // TODO test
            public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
            public static final Translation2d VISION_LIMIT = new Translation2d(Units.feetToMeters(9), Units.feetToMeters(5));
    }
}
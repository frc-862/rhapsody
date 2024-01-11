// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.subsystems.Swerve;
import frc.thunder.math.InterpolationMap;

/** Add your docs here. */
public class Constants {
    public class drivetrainConstants { //TODO Get new for new robot
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches*2; //2 * x distance from center of robot to wheel
        public static final double MaxAngularRate = 2*Math.PI*( //convert to radians per second
                TunerConstants.kSpeedAt12VoltsMps / // free speed
                Math.PI*Math.sqrt(2*Math.pow(WHEELBASE, 2)) // circumference of circle with radius of wheelbase
        );
		public static final double RotationMultipler = 0.5; // TODO tune
    }

    public class RobotMap {
        public class CAN {
            // Front Left
            private static final int kFrontLeftDriveMotorId = 1;
        	private static final int kFrontLeftSteerMotorId = 2;
        	private static final int kFrontLeftEncoderId = 31;

			// Front Right
        	private static final int kFrontRightDriveMotorId = 3;
        	private static final int kFrontRightSteerMotorId = 4;
        	private static final int kFrontRightEncoderId = 33;

			// Back Left
			private static final int kBackLeftDriveMotorId = 7;
			private static final int kBackLeftSteerMotorId = 8;
			private static final int kBackLeftEncoderId = 34;

			// Back Right
        	private static final int kBackRightDriveMotorId = 5;
        	private static final int kBackRightSteerMotorId = 6;
        	private static final int kBackRightEncoderId = 32;

			public static final int PigeonId = 23;            
            public static final int COLLECTOR_MOTOR = 9;
            public static final int FLYWHEEL_MOTOR_1 = 0;
            public static final int FLYWHEEL_MOTOR_2 = 0;
            public static final int SHOOTER_ANGLE_MOTOR = 0;

        }

        public static final int COLLECTOR_ENTRY_BEAMBREAK = 1;
    }

    public static class ControllerConstants {
		public static final int DriverControllerPort = 0;
		public static final int CopilotControllerPort = 1;
	}

     public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot
        // The steer motor uses MotionMagicVoltage control
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.05)
            .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses:
        // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
        // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);
    
        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;
    
        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        private static final double kSpeedAt12VoltsMps = 6.0;
    
        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;
    
        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 2;
    
        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;
    
        private static final String kCANbusName = "Canivore";
        public static final int kPigeonId = 23;
    
    
        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
    
        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);
    
        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);
    
		// OFFSETS
        private static final double kFrontLeftEncoderOffset = -0.22314453125;
        private static final double kFrontLeftXPosInches = 11.25;
        private static final double kFrontLeftYPosInches = 11.25;
    
        private static final double kFrontRightEncoderOffset = 0.379150390625;
        private static final double kFrontRightXPosInches = 11.25;
        private static final double kFrontRightYPosInches = -11.25;
        
        private static final double kBackLeftEncoderOffset = 0.133056640625;
        private static final double kBackLeftXPosInches = -11.25;
        private static final double kBackLeftYPosInches = 11.25;
    
        private static final double kBackRightEncoderOffset = -0.173828125;
        private static final double kBackRightXPosInches = -11.25;
        private static final double kBackRightYPosInches = -11.25;
    
    
        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                CAN.kFrontLeftSteerMotorId, CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                CAN.kFrontRightSteerMotorId, CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                CAN.kBackLeftSteerMotorId, CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                CAN.kBackRightSteerMotorId, CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
    
        public static final Swerve DriveTrain = new Swerve(DrivetrainConstants, FrontLeft,
                FrontRight, BackLeft, BackRight);
    }

    public class VisionConstants  {
         //This is a magic number from gridlock, may need to be changed or removed entirely
            public static final double PROCESS_LATENCY = 0.0472; // TODO test
            public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
            public static final Translation2d VISION_LIMIT = new Translation2d(Units.feetToMeters(9), Units.feetToMeters(5));
    }
    
    public class FlywheelConstants {
        public static final boolean FLYWHEEL_MOTOR_1_INVERT = false;
        public static final boolean FLYWHEEL_MOTOR_2_INVERT = false;
        public static final int FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final NeutralModeValue FLYWHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double FLYWHEEL_MOTOR_KP = 0;
        public static final double FLYWHEEL_MOTOR_KI = 0;
        public static final double FLYWHEEL_MOTOR_KD = 0;
        public static final double FLYWHEEL_MOTOR_KS = 0;
        public static final double FLYWHEEL_MOTOR_KV = 0;

        public static final double FLYWHEEL_RPM_TOLERANCE = 0;
        // Distance in meters, angle in degrees
        public static final InterpolationMap SHOOTER_DISTANCE_ANGLE_MAP = new InterpolationMap();
        // Distance in meters, speed in RPM
        public static final InterpolationMap SHOOTER_DISTANCE_SPEED_MAP = new InterpolationMap();
    }

    public class PivotConstants {
        public static final boolean PIVOT_MOTOR_INVERT = false;
        public static final int PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int PIVOT_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final NeutralModeValue PIVOT_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double PIVOT_MOTOR_KP = 0;
        public static final double PIVOT_MOTOR_KI = 0;
        public static final double PIVOT_MOTOR_KD = 0;        
        public static final double PIVOT_MOTOR_KS = 0;
        public static final double PIVOT_MOTOR_KV = 0;
        public static final double PIVOT_TOLERANCE = 0;

    }
}
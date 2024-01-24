package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.subsystems.Swerve;
import frc.thunder.math.InterpolationMap;

/** Add your docs here. */
public class Constants {

    public static final Path MERCURY_PATH = Paths.get("/home/lvuser/mercury");

    public static final boolean isMercury() {
        return MERCURY_PATH.toFile().exists();
    }

    public class DrivetrAinConstants { // TODO Get new for new robot
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches * 2; // 2 * x distance from center of robot to wheel
        public static final double MaxAngularRate = 2 * Math.PI * ( // convert to radians per second
        TunerConstants.kSpeedAt12VoltsMps / // free speed
                Math.PI * Math.sqrt(2 * Math.pow(WHEELBASE, 2)) // circumference of circle with radius of wheelbase
        );
        // TODO: TUNE
        public static final double ROT_MULT = 0.015; // TODO Tune for Driver

        public static final double SLOW_ROT_MULT = 0.007; // TODO Tune for Driver
        public static final double SLOW_SPEED_MULT = 0.4; // TODO Tune for Driver
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
            private static final int kFrontRightEncoderId = 32;

            // Back Left
            private static final int kBackLeftDriveMotorId = 7;
            private static final int kBackLeftSteerMotorId = 8;
            private static final int kBackLeftEncoderId = 34;

            // Back Right
            private static final int kBackRightDriveMotorId = 5;
            private static final int kBackRightSteerMotorId = 6;
            private static final int kBackRightEncoderId = 33;

            public static final int PigeonId = 23;
            
            // TODO Get real temp so less errors
            public static final int COLLECTOR_MOTOR_TOP = 9;
            public static final int COLLECTOR_MOTOR_BACK = 10;
            public static final int FLYWHEEL_MOTOR_1 = 11; 
            public static final int FLYWHEEL_MOTOR_2 = 12; 
            public static final int SHOOTER_ANGLE_MOTOR = 13; 
            public static final int INDEXER_MOTOR = 14;

            public static final String CANBUS = "Canivore";
        }

        /**
         * You expected a javadoc, but it was me, Dio!
         */
        public class DIO {
            public static final int COLLECTOR_ENTRY_BEAMBREAK_FRONT = 1;
            public static final int COLLECTOR_ENTRY_BEAMBREAK_BACK = 2;
            public static final int INDEXER_BEAMBREAK = 0;
        }
    }

    public static class ControllerConstants {
        public static final int DriverControllerPort = 0;
        public static final int CopilotControllerPort = 1;

        public static final double DEADBAND = 0.1;
    }

    public static class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0); // TODO: Tune
        public static final PIDConstants ROTATION_PID = new PIDConstants(10, 0, 0); // TODO: Tune

        public static final double MAX_MODULE_VELOCITY = Units.feetToMeters(17.3); // f/s to m/s
        public static final double DRIVE_BASE_RADIUS = Units.feetToMeters(19.09); // TODO check

        public static final double CONTROL_LOOP_PERIOD = 0.004; // IS this right?
    }

    public static class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs().withKP(100).withKI(0)
                .withKD(0.2).withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains =
                new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput =
                ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput =
                ClosedLoopOutputType.TorqueCurrentFOC;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300d;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.21;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 23;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants =
                new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator =
                new SwerveModuleConstantsFactory().withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches).withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains).withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps).withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

        // OFFSETS Rhapsody
        private static final double kFrontLeftEncoderOffsetRh = 0.0439453125;
        private static final double kFrontLeftXPosInchesRh = 13.5;
        private static final double kFrontLeftYPosInchesRh = 13.5;

        private static final double kFrontRightEncoderOffsetRh = 0.288818359375;
        private static final double kFrontRightXPosInchesRh = 13.5;
        private static final double kFrontRightYPosInchesRh = -13.5;

        private static final double kBackLeftEncoderOffsetRh = 0.2197265625;
        private static final double kBackLeftXPosInchesRh = -13.5;
        private static final double kBackLeftYPosInchesRh = 13.5;

        private static final double kBackRightEncoderOffsetRh = 0.0009765625;
        private static final double kBackRightXPosInchesRh = -13.5;
        private static final double kBackRightYPosInchesRh = -13.5;

        // OFFSETS Mercury
        private static final double kFrontLeftEncoderOffset = -0.23876953125;
        private static final double kFrontLeftXPosInches = 13.5;
        private static final double kFrontLeftYPosInches = 13.5;

        private static final double kFrontRightEncoderOffset = 0.391845703125;
        private static final double kFrontRightXPosInches = 13.5;
        private static final double kFrontRightYPosInches = -13.5;

        private static final double kBackLeftEncoderOffset = 0.13916015625;
        private static final double kBackLeftXPosInches = -13.5;
        private static final double kBackLeftYPosInches = 13.5;

        private static final double kBackRightEncoderOffset = -0.23388671875;
        private static final double kBackRightXPosInches = -13.5;
        private static final double kBackRightYPosInches = -13.5;



        private static final SwerveModuleConstants FrontLeft =
                ConstantCreator.createModuleConstants(CAN.kFrontLeftSteerMotorId,
                        CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches),
                        Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight =
                ConstantCreator.createModuleConstants(CAN.kFrontRightSteerMotorId,
                        CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId,
                        kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                CAN.kBackLeftSteerMotorId, CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId,
                kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches),
                Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight =
                ConstantCreator.createModuleConstants(CAN.kBackRightSteerMotorId,
                        CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId,
                        kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches),
                        Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

        private static final SwerveModuleConstants FrontLeftRh =
                ConstantCreator.createModuleConstants(CAN.kFrontLeftSteerMotorId,
                        CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId,
                        kFrontLeftEncoderOffsetRh, Units.inchesToMeters(kFrontLeftXPosInchesRh),
                        Units.inchesToMeters(kFrontLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRightRh =
                ConstantCreator.createModuleConstants(CAN.kFrontRightSteerMotorId,
                        CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId,
                        kFrontRightEncoderOffsetRh, Units.inchesToMeters(kFrontRightXPosInchesRh),
                        Units.inchesToMeters(kFrontRightYPosInchesRh), kInvertRightSide);
        private static final SwerveModuleConstants BackLeftRh =
                ConstantCreator.createModuleConstants(CAN.kBackLeftSteerMotorId,
                        CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId, kBackLeftEncoderOffsetRh,
                        Units.inchesToMeters(kBackLeftXPosInchesRh),
                        Units.inchesToMeters(kBackLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants BackRightRh =
                ConstantCreator.createModuleConstants(CAN.kBackRightSteerMotorId,
                        CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId,
                        kBackRightEncoderOffsetRh, Units.inchesToMeters(kBackRightXPosInchesRh),
                        Units.inchesToMeters(kBackRightYPosInchesRh), kInvertRightSide);


        public static final Swerve getDrivetrain() {
            if (Constants.isMercury()) {
                System.out.println("IS MERCURY");
                return new Swerve(DrivetrainConstants, 250, FrontLeft, FrontRight, BackLeft,
                        BackRight);
            } else {
                System.out.println("IS RHAPSODY");
                return new Swerve(DrivetrainConstants, 250, FrontLeftRh, FrontRightRh, BackLeftRh,
                        BackRightRh);
            }
        }
    }

    public class VisionConstants {
        // This is a magic number from gridlock, may need to be changed or removed entirely
        public static final double PROCESS_LATENCY = 0.0472; // TODO test
        public static final Translation2d FIELD_LIMIT =
                new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
        public static final Translation2d VISION_LIMIT =
                new Translation2d(Units.feetToMeters(9), Units.feetToMeters(5));
        public static final double COLLISION_DEADZONE = 2d;
        public static final double ALIGNMENT_TOLERANCE = 4d; // TODO: make this an actual value
        public static final PIDController HEADING_CONTROLLER = new PIDController(0.05, 0, 0);
    }

    public class CollectorConstants {
        public static final boolean COLLECTOR_MOTOR_INVERTED_TOP = false; //TODO check once collector installed
        public static final int COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_TOP = 0; // TODO: make sure they are not set to 0
        public static final int COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_TOP = 0; // TODO: make sure they are not set to 0
        public static final NeutralModeValue COLLECTOR_MOTOR_NEUTRAL_MODE_TOP = NeutralModeValue.Coast;

        public static final boolean COLLECTOR_MOTOR_INVERTED_BOTTOM = false; //TODO check once collector installed
        public static final int COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT_BOTTOM = 0; // TODO: make sure they are not set to 0
        public static final int COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT_BOTTOM = 0; // TODO: make sure they are not set to 0
        public static final NeutralModeValue COLLECTOR_MOTOR_NEUTRAL_MODE_BOTTOM = NeutralModeValue.Coast;
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

        public static final double RPM_TOLERANCE = 0;
    }

    public class IndexerConstants {
        public static final boolean INDEXER_MOTOR_INVERTED = false;
        public static final int INDEXER_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int INDEXER_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final NeutralModeValue INDEXER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double INDEXER_DEFAULT_POWER = 0.3; //TODO: get real
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

        public static final double ANGLE_TOLERANCE = 0;

    }

    public class ShooterConstants {
        public static final double BASE_RPM = 0;
        public static final double STOW_ANGLE = 0;
        
        public static final double FAR_WING_X = 3.3;

        public enum SHOOTER_STATES {
            STOW, PRIME, AIM, SHOOT
        }

        // Distance in meters, angle in degrees
        public static final InterpolationMap ANGLE_MAP = new InterpolationMap() {
            {
                put(0d, 0d);
            }
        };

        // Distance in meters, speed in RPM
        public static final InterpolationMap SPEED_MAP = new InterpolationMap() {
            {
                put(0d, 0d);
            }
        };
    }
}

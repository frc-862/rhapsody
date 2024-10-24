package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.subsystems.Swerve;
import frc.thunder.math.InterpolationMap;

public class Constants {

    public static final Path MERCURY_PATH = Paths.get("/home/lvuser/mercury");

    public static final boolean IS_MERCURY = MERCURY_PATH.toFile().exists();

    public static final String HOOT_PATH = "U/logs"; // "/home/lvuser/logs";

    public class DrivetrainConstants {
        public static final double MaxSpeed = Units.feetToMeters(18); // 16.5 ft/s to meters per second top speed (5.0292m/s)
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches * 2; // 2 * x distance from center of robot to wheel
        public static final double MaxAngularRate = 2 * Math.PI * ( // convert to radians per second
        TunerConstants.kSpeedAt12VoltsMps / Math.PI * Math.sqrt(2 * Math.pow(WHEELBASE, 2))); // free speed / circumference of circle with radius of wheelbase

        public static final double ROT_MULT = 0.04; // TODO Tune for Driver

        public static final double SLOW_ROT_MULT = 0.7;
        public static final double SLOW_SPEED_MULT = 0.4;

        public static final double SYS_TEST_SPEED_DRIVE = 0.5;
        public static final double SYS_TEST_SPEED_TURN = 0.7d;

        public static final Translation2d SPEAKER_POSE = new Translation2d(0d, 5.547393);
        public static final Translation2d RED_SPEAKER_POSE = new Translation2d(15.6592d, 5.547393d);
        public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(0.8d, 5.547393d);

        public static final Translation2d RED_CORNER_POSE = new Translation2d(15.6592d, 7.5d);
        public static final Translation2d BLUE_CORNER_POSE = new Translation2d(0.8d, 7.5d); // TODO get real >:)


        public static final double ALIGNMENT_TOLERANCE = 1d;
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

            public static final int COLLECTOR_MOTOR = 9;
            public static final int INDEXER_MOTOR = 10;
            public static final int PIVOT_ANGLE_MOTOR = 11;
            public static final int FLYWHEEL_MOTOR_BOTTOM = 12;
            public static final int FLYWHEEL_MOTOR_TOP = 13;
            public static final int CLIMB_RIGHT = 14;
            public static final int CLIMB_LEFT = 15;

            // Cancoders
            public static final int PIVOT_ANGLE_CANCODER = 35;

            public static final String CANBUS_FD = "Canivore";
            public static final String RIO_CANBUS = "rio";
        }

        /**
         * You expected a javadoc, but it was me, Dio!
         */
        public class DIO {
            public static final int COLLECTOR_BEAMBREAK = 2;
            public static final int INDEXER_ENTER_BEAMBREAK = 0;
            public static final int INDEXER_EXIT_BEAMBREAK = 1;

        }

        public class PWM {
            public static final int LED_PORT = 0;
        }
    }

    public static class ControllerConstants {
        public static final int DriverControllerPort = 0;
        public static final int CopilotControllerPort = 1;
        public static final int ButtonBoxControllerPort = 2;

        public static final double DEADBAND = 0.1;

        public static class ButtonBox {
            public static final int GRAY_TOPLEFT = 5;
            public static final int PINK = 3;
            public static final int GREEN = 4;
            public static final int GRAY_TOPRIGHT = 6;
            public static final int GRAY_BOTTOMLEFT = 2; // AXIS
            public static final int PURPLE = 1;
            public static final int RED = 2;
            public static final int GRAY_BOTTOMRIGHT = 3; // AXIS
            public static final int SHARE = 7;
            public static final int OPTIONS = 8;
            public static final int L3_SL = 9;
            public static final int R3_SR = 10;
        }
    }

    public static class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

        public static final double MAX_MODULE_VELOCITY = Units.feetToMeters(16.5); // f/s to m/s
        public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(10.825);

        public static final double CONTROL_LOOP_PERIOD = 0.02;

        public static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, false); // Expirement with dynamic replaning in offseason
        public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(2.0, 1.0, 3.0, 1.5);
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2.0, 1, 1.0, 0.5);

        public static final Pose2d AMP_LOCATION_RED = new Pose2d(new Translation2d(14.4, 7.62), new Rotation2d(90));
    }

    public static class ChaseConstants {
        public static final double BLUE_CHASE_BOUNDARY = 8.35; // The highest X value the robot can be at before ending. Prevents going over center line.
        public static final double RED_CHASE_BOUNDARY = 8.25;

        public static final double BLUE_SLOW_CHASE_RANGE = 8.05;
        public static final double RED_SLOW_CHASE_RANGE = 8.55;
        public static final double CHASE_BOUNDARY = 8.3; // The highest X value the robot can be at before ending. Prevents going over center line.

        public static final PIDController CHASE_CONTROLLER = new PIDController(0.05, 0, 0);
        public static final double CHASE_PIECE_ALIGNMENT_TOLERANCE = 3d;
    }

    public static class PassConstants {
        public static final double POINT_TOLERANCE = 3d;

        public static final PIDController PASS_CONTROLLER = new PIDController(0.1, 0, 0.01);
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(0.25, 0.5);
    }

    public static class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 60d;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.02;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        public static final double kDriveGearRatio = 6.122448979591837;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kWheelRadiusInches = 1.95;

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

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory().withDriveMotorGearRatio(kDriveGearRatio).withSteerMotorGearRatio(kSteerGearRatio).withWheelRadius(kWheelRadiusInches).withSlipCurrent(kSlipCurrentA).withSteerMotorGains(steerGains).withDriveMotorGains(driveGains).withSteerMotorClosedLoopOutput(steerClosedLoopOutput).withDriveMotorClosedLoopOutput(driveClosedLoopOutput).withSpeedAt12VoltsMps(kSpeedAt12VoltsMps).withSteerInertia(kSteerInertia).withDriveInertia(kDriveInertia).withSteerFrictionVoltage(kSteerFrictionVoltage).withDriveFrictionVoltage(kDriveFrictionVoltage).withFeedbackSource(SteerFeedbackType.FusedCANcoder).withCouplingGearRatio(kCoupleRatio).withSteerMotorInverted(kSteerMotorReversed);

        // OFFSETS Rhapsody
        private static final double kFrontLeftEncoderOffsetRh = 0.046142578125;
        private static final double kFrontLeftXPosInchesRh = 10.825;
        private static final double kFrontLeftYPosInchesRh = 10.825;

        private static final double kFrontRightEncoderOffsetRh = 0.29931640625;
        private static final double kFrontRightXPosInchesRh = 10.825;
        private static final double kFrontRightYPosInchesRh = -10.825;

        private static final double kBackLeftEncoderOffsetRh = 0.196533203125;
        private static final double kBackLeftXPosInchesRh = -10.825;
        private static final double kBackLeftYPosInchesRh = 10.825;

        private static final double kBackRightEncoderOffsetRh = -0.03662109375;
        private static final double kBackRightXPosInchesRh = -10.825;
        private static final double kBackRightYPosInchesRh = -10.825;

        // OFFSETS Mercury
        private static final double kFrontLeftEncoderOffset = -0.11572265625;
        private static final double kFrontLeftXPosInches = 10.825;
        private static final double kFrontLeftYPosInches = 10.825;

        private static final double kFrontRightEncoderOffset = 0.01220703125;
        private static final double kFrontRightXPosInches = 10.825;
        private static final double kFrontRightYPosInches = -10.825;

        private static final double kBackLeftEncoderOffset = 0.032958984375;
        private static final double kBackLeftXPosInches = -10.825;
        private static final double kBackLeftYPosInches = 10.825;

        private static final double kBackRightEncoderOffset = 0.13330078125;
        private static final double kBackRightXPosInches = -10.825;
        private static final double kBackRightYPosInches = -10.825;

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(CAN.kFrontLeftSteerMotorId, CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(CAN.kFrontRightSteerMotorId, CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(CAN.kBackLeftSteerMotorId, CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(CAN.kBackRightSteerMotorId, CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

        private static final SwerveModuleConstants FrontLeftRh = ConstantCreator.createModuleConstants(CAN.kFrontLeftSteerMotorId, CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId, kFrontLeftEncoderOffsetRh, Units.inchesToMeters(kFrontLeftXPosInchesRh), Units.inchesToMeters(kFrontLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRightRh = ConstantCreator.createModuleConstants(CAN.kFrontRightSteerMotorId, CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId, kFrontRightEncoderOffsetRh, Units.inchesToMeters(kFrontRightXPosInchesRh), Units.inchesToMeters(kFrontRightYPosInchesRh), kInvertRightSide);
        private static final SwerveModuleConstants BackLeftRh = ConstantCreator.createModuleConstants(CAN.kBackLeftSteerMotorId, CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId, kBackLeftEncoderOffsetRh, Units.inchesToMeters(kBackLeftXPosInchesRh), Units.inchesToMeters(kBackLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants BackRightRh = ConstantCreator.createModuleConstants(CAN.kBackRightSteerMotorId, CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId, kBackRightEncoderOffsetRh, Units.inchesToMeters(kBackRightXPosInchesRh), Units.inchesToMeters(kBackRightYPosInchesRh), kInvertRightSide);

        public static final Swerve getDrivetrain() {
            if (Constants.IS_MERCURY) {
                System.out.println("IS MERCURY");
                return new Swerve(DrivetrainConstants, 250, FrontLeft, FrontRight, BackLeft, BackRight);
            } else {
                System.out.println("IS RHAPSODY");
                return new Swerve(DrivetrainConstants, 250, FrontLeftRh, FrontRightRh, BackLeftRh, BackRightRh);
            }
        }
    }

    public class VisionConstants {
        // This is a magic number from LL2+, may need to be changed or removed entirely
        public static final double PROCESS_LATENCY = 0.0472; // TODO test this value

        public static final double POINTATTAG_ALIGNMENT_TOLERANCE = 1d;
        public static final double POINTATPOINT_ALIGNMENT_TOLERANCE = 1d;
        public static final PIDController POINT_AIM_CONTROLLER = new PIDController(0.2, 0, 0.015, 0.01);
        public static final PIDController TAG_AIM_CONTROLLER = new PIDController(0.1, 0, 0.01);
        public static final PIDController COMBO_CONTROLLER = new PIDController(0.1, 0, 0.01);

        public static final int[] SPEAKER_FILTERS = {4, 7};
        public static final int[] ALL_TAG_FILTERS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};


        public static class LimelightOrientation {
            public static final double STOPME_YAW = 180d;
            public static final double STOPME_PITCH = 55.5;
            public static final double STOPME_ROLL = 0d;
        }

        public class Pipelines {
            public static final int TAG_PIPELINE = 0;
            public static final int SPEAKER_PIPELINE = 1; // Not currently in use, using feducial filtering instead
            public static final int NOTE_PIPELINE = 0;
        }
    }

    public class PoseConstants {
        public static class StartingPoseConstants {
            public static final Pose2d SOURCE_SUB_A_STARTPOSE_BLUE = new Pose2d(new Translation2d(0.72, 6.69), new Rotation2d(60));
            public static final Pose2d SOURCE_SUB_B_STARTPOSE_BLUE = new Pose2d(new Translation2d(1.34, 5.55), new Rotation2d(0));
            public static final Pose2d SOURCE_SUB_C_STARTPOSE_BLUE = new Pose2d(new Translation2d(0.72, 4.39), new Rotation2d(-60));

            public static final Pose2d SOURCE_SUB_A_STARTPOSE_RED = new Pose2d(new Translation2d(15.85, 6.70), new Rotation2d(120));
            public static final Pose2d SOURCE_SUB_B_STARTPOSE_RED = new Pose2d(new Translation2d(15.2, 5.55), new Rotation2d(180));
            public static final Pose2d SOURCE_SUB_C_STARTPOSE_RED = new Pose2d(new Translation2d(15.85, 4.35), new Rotation2d(-120));
        }

        public static class PathfindPoseConstants {
            public static final Pose2d PATHFIND_CENTER_STAGE_START_POSE_BLUE = new Pose2d(7.43, 4.16, new Rotation2d(0));
            public static final Pose2d PATHFIND_HIGH_STAGE_START_POSE_BLUE = new Pose2d(3.55, 6.16, new Rotation2d(Units.degreesToRadians(-53.13)));
            public static final Pose2d PATHFIND_LOW_STAGE_START_POSE_BLUE = new Pose2d(3.54, 1.91, new Rotation2d(Units.degreesToRadians(63.43)));

            public static final Pose2d PATHFIND_CENTER_STAGE_START_POSE_RED = new Pose2d(9.07, 4.16, new Rotation2d(0));
            public static final Pose2d PATHFIND_HIGH_STAGE_START_POSE_RED = new Pose2d(12.96, 6.16, new Rotation2d(Units.degreesToRadians(-53.13)));
            public static final Pose2d PATHFIND_LOW_STAGE_START_POSE_RED = new Pose2d(12.95, 1.91, new Rotation2d(Units.degreesToRadians(63.43)));
        }

        public static final double FAR_WING_X = 3.3;

        public static final Translation3d BLUE_SPEAKER_LOCATION = new Translation3d(0, 5.547593, 1.2);
        public static final Translation3d RED_SPEAKER_LOCATION = new Translation3d(16.4592, 5.547593, 1.2);

        public static final double HALF_FIELD_HEIGHT = Units.feetToMeters(13);
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
    }

    public class PathFindingConstants {
        public static final Pose2d TEST_POSE = new Pose2d(9, 4, new Rotation2d(90));
        public static final Translation2d RED_ORIGIN = new Translation2d(16.4592, 0);
    }

    public class CollisionConstants {
        public static final double TIP_DEADZONE = 2d;

        public static final double ACCELERATION_DUE_TO_GRAVITY = 9.80665;
        public static final double ACCELERATION_TOLERANCE_TELEOP = 3; // percent of pigeonAcceleration
        public static final double MIN_ACCELERATION_DIFF_TELEOP = 0.25; // TODO: get real
        public static final double ACCELERATION_TOLERANCE_AUTON = 2.00; // percent of pigeonAcceleration
        public static final double MIN_ACCELERATION_DIFF_AUTON = 0.25; // TODO: get real
        public static final double ACCELERATION_TOLERANCE_SHOOTER = 1.00; // percent of pigeonAcceleration
        public static final double MIN_ACCELERATION_DIFF_SHOOTER = 0.25; // TODO: get real

        public enum CollisionType {
            AUTON, TELEOP, SHOOTER
        }
    }

    public class MusicConstants {
        public static final String BOH_RHAP_FILEPATH = "bohemianrhapsody.chrp";
        public static final String JEOPARDY_FILEPATH = "jeopardy.chrp";
        public static final String BEWARE_THE_FOREST_MUSHROOMS_FILEPATH = "bewaretheforestmushrooms.chrp";
        public static final String UNDER_PRESSURE_FILEPATH = "underpressure.chrp";
        public static final String NATIONAL_PARK_FILEPATH = "nationalpark.chrp";
        public static final String ENCOUNTER_FILEPATH = "encounter.chrp";
        public static final String PIRATES_OF_THE_CARIBBEAN_FILEPATH = "piratesofthecaribbean.chrp";
        public static final String CRAZY_LITTLE_THING_CALLED_LOVE_FILEPATH = "crazylittlethingcalledlove.chrp";
        public static final String ANOTHER_ONE_BITES_THE_DUST_FILEPATH = "anotheronebitesthedust.chrp";
        public static final String SWEET_CAROLINE_FILEPATH = "sweetcaroline.chrp";
        public static final String WE_ARE_THE_CHAMPIONS_FILEPATH = "wearethechampions.chrp";
        public static final String[] SONG_NAMES = {BOH_RHAP_FILEPATH, JEOPARDY_FILEPATH, BEWARE_THE_FOREST_MUSHROOMS_FILEPATH, UNDER_PRESSURE_FILEPATH, NATIONAL_PARK_FILEPATH, ENCOUNTER_FILEPATH, PIRATES_OF_THE_CARIBBEAN_FILEPATH, CRAZY_LITTLE_THING_CALLED_LOVE_FILEPATH, ANOTHER_ONE_BITES_THE_DUST_FILEPATH, SWEET_CAROLINE_FILEPATH, WE_ARE_THE_CHAMPIONS_FILEPATH};
        public static final List<String> SET_LIST = Arrays.asList(SONG_NAMES);
    }

    public class CollectorConstants {
        public static final boolean COLLECTOR_MOTOR_INVERTED = true;
        public static final int COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT = 140;
        public static final boolean COLLECTOR_MOTOR_BRAKE_MODE = false;

        public static final double COLLECTOR_DEBOUNCE_TIME = 0.5d;

        public static final double MOTOR_KP = 0;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0;
        public static final double MOTOR_KV = 0.145;
        public static final double MOTOR_KA = 0;

        public static final double COLLECTOR_SYSTEST_POWER = 0.25;
        public static final double COLLECTOR_GRABANDGO_POWER = 0.75;
    }

    public class FlywheelConstants {
        public static final boolean MOTOR_TOP_INVERT_Rhapsody = false;
        public static final boolean MOTOR_TOP_INVERT_Mercury = true;

        public static final boolean MOTOR_BOTTOM_INVERT = false;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 80;
        public static final boolean MOTOR_BRAKE_MODE = false;

        // SLOT 0 TOP, 0 - 49 RPS
        public static final double TOP_0_MOTOR_KP = 0.2;
        public static final double TOP_0_MOTOR_KI = 0.07;
        public static final double TOP_0_MOTOR_KD = 0;
        public static final double TOP_0_MOTOR_KS = 0.26;
        public static final double TOP_0_MOTOR_KV = 0.11;
        public static final double TOP_0_MOTOR_KA = 2.9;

        // SLOT 1 TOP, 50 - 107 RPS
        public static final double TOP_1_MOTOR_KP = 0.163;
        public static final double TOP_1_MOTOR_KI = 0;
        public static final double TOP_1_MOTOR_KD = 0;
        public static final double TOP_1_MOTOR_KS = 0.3;
        public static final double TOP_1_MOTOR_KV = 0.114;
        public static final double TOP_1_MOTOR_KA = 0;

        // SLOT 0 BOTTOM, 0 - 49 RPS
        public static final double BOTTOM_0_MOTOR_KP = 0.2;
        public static final double BOTTOM_0_MOTOR_KI = 0.07;
        public static final double BOTTOM_0_MOTOR_KD = 0;
        public static final double BOTTOM_0_MOTOR_KS = 0.26;
        public static final double BOTTOM_0_MOTOR_KV = 0.11;
        public static final double BOTTOM_0_MOTOR_KA = 2.9;

        // SLOT 1 BOTTOM, 50 - 107 RPS
        public static final double BOTTOM_1_MOTOR_KP = 0.15;
        public static final double BOTTOM_1_MOTOR_KI = 0;
        public static final double BOTTOM_1_MOTOR_KD = 0;
        public static final double BOTTOM_1_MOTOR_KS = 0.35;
        public static final double BOTTOM_1_MOTOR_KV = 0.114;
        public static final double BOTTOM_1_MOTOR_KA = 0;

        public static final double RPM_TOLERANCE = 100d;

        public static final double BIAS_INCREMENT = 1.25; // RPS to bias by per button press
        public static final double COAST_VOLTAGE = 0.1;

        public static final double FLYWHEEL_SYSTEST_RPM = 1000;
    }

    public class IndexerConstants {
        public static final boolean MOTOR_INVERT = true;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 170;

        public enum PieceState {
            IN_COLLECT, IN_PIVOT, IN_INDEXER, NONE
        }

        public static final boolean INDEXER_MOTOR_BRAKE_MODE = true;

        public static final double INDEXER_DEFAULT_POWER = 1d;
        public static final double INDEXER_MANUAL_POWER = 1d;
        public static final double INDEXER_DEBOUNCE_TIME = 0.25d;
        public static final double INDEXER_SYSTEST_POWER = 0.25d;
    }

    public class MercuryPivotConstants {
        public static final boolean MOTOR_INVERT = true; // POS power is up
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean MOTOR_BRAKE_MODE = true;
        public static final double MOTOR_KP = 0;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0;
        public static final double MOTOR_KV = 3;
        public static final double MOTOR_KA = 0;

        // Not currently using Motion magic
        public static final double MAGIC_CRUISE_VEL = 0.01;
        public static final double MAGIC_ACCEL = 0.02;
        public static final double MAGIC_JERK = 0.2;

        public static final double ANGLE_TOLERANCE = 0.00208d;

        public static final double ENCODER_OFFSET = 0.61095; // In rotations
        public static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;
        public static final double ROTOR_TO_ENCODER_RATIO = 618.75;

        public static final double BIAS_INCREMENT = 1d; // Degrees to bias by per button press

        public static final double STOW_ANGLE = 28d;

        public static final double MAX_INDEX_ANGLE = 40d;

        public static final double MIN_ANGLE = 25d;
        public static final double MAX_ANGLE = 105d;

        public static final double PIVOT_SYSTEST_ANGLE = 90d;
    }

    public class RhapsodyPivotConstants {
        public static final boolean MOTOR_INVERT = true; // POS power is up
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean MOTOR_BRAKE_MODE = true;
        public static final double MOTOR_KP = 190;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KG = 0.393;
        public static final double MOTOR_KV = 50d;
        public static final double MOTOR_KS = 20d;
        public static final double MOTOR_KA = 0d;

        public static final double ANGLE_TOLERANCE = 0.00208d; // .75 degrees

        public static final double ENCODER_OFFSET = -0.913834; // In rotations
        public static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;
        public static final double ROTOR_TO_ENCODER_RATIO = 275d;

        public static final double BIAS_INCREMENT = 0.5d; // Bias by .5 degrees

        public static final double STOW_ANGLE = 27d;

        public static final double MAX_INDEX_ANGLE = 40d;

        public static final double MIN_ANGLE = 25d; // TODO get new value
        public static final double MAX_ANGLE = 105d;

        public static final double PIVOT_SYSTEST_ANGLE = 90d;
    }

    public class ShooterConstants {

        // Distance in meters, angle in degrees
        public static final InterpolationMap TUBE_ANGLE_MAP = new InterpolationMap() {
            {
                // As distance gets smaller angle goes up
                put(1.21d, 52d);
                put(2d, 45d);
                put(2.5d, 41.5d);
                put(3d, 37d);
                put(3.5d, 30d);
                put(4.09d, 26.5d);
                put(4.86, 26.5d);
            }
        };

        // Distance in meters, speed in RPM
        public static final InterpolationMap TUBE_SPEED_MAP = new InterpolationMap() {
            {
                // As distance get smaller RPM gets smaller
                put(1.21d, 2000d);
                put(2d, 2500d);
                put(2.5d, 3000d);
                put(3d, 3500d);
                put(3.5d, 4000d);
                put(4d, 3600d);
                put(4.75d, 4600d);
            }
        };

        // Distance in meters, angle in degrees
        public static final InterpolationMap STEALTH_ANGLE_MAP = new InterpolationMap() {
            {
                // As distance gets smaller angle goes up
                put(1.21d, 50d);
                put(1.67d, 40d);
                put(1.819, 38d);
                // switch
                put(1.82, 34d);
                // linear
                put(2.5d, 34d);
                put(2.6d, 32d);
                put(2.7, 31d);
                put(3d, 30.0d);
                put(3.08, 29d);
                put(3.15, 28d);
                put(3.23, 28d);
                put(3.51d, 27d);
                // end of travel
                put(4.0d, 26d);
                put(4.33, 26d);
                // put(4.71d, 26d);
                // put(6.2d, 23.5d);
            }
        };

        // Distance in meters, speed in RPM
        public static final InterpolationMap STEALTH_SPEED_MAP = new InterpolationMap() {
            {
                // As distance get smaller RPM gets smaller
                put(1.21d, 2000d);
                put(1.64d, 2500d);
                put(1.819, 3000d);
                // switch
                put(1.82, 4000d);
                // linear
                put(3.51d, 4000d);
                // end of travel
                put(4.0d, 3500d);
                put(4.33d, 3000d);
                // put(4.71d, 5500d);
                // put(6.2d, 5800d);
            }
        };

        public static final InterpolationMap NOTEPASS_ANGLE_MAP = new InterpolationMap() {
            {
                // put(13.716, 36d);
                // put(12.192, 36d);
                // put(10.668, 37d);
                // put(9.144, 38d);
                // put(7.62, 41d);
                // put(6.096, 46d);
                put(10.1d, 38d);
                put(11.03d, 36d);
            }
        };

        public static final InterpolationMap NOTEPASS_SPEED_MAP = new InterpolationMap() {
            {
                // put(13.716, 3600d);
                // put(12.192, 3300d);
                // put(20.668, 2900d);
                // put(9.144, 2600d);
                // put(7.62, 2250d);
                // put(6.096, 1900d);
                put(10.1d, 5500d);
                put(11.03d, 6000d);
            }
        };

        public enum ShootingState {
            AIM, SHOOT, SHOT
        }

        public static final double TIME_TO_SHOOT = 1d; // Time in seconds it takes from indexer start to flywheel exit
    }

    public class CandConstants {
        // Amp
        public static final double AMP_TOP_RPM = 500;
        public static final double AMP_BOTTOM_RPM = 700;
        public static final double AMP_ANGLE = 94;

        // PointBlank
        public static final double POINT_BLANK_RPM = 2500;
        public static final double POINT_BLANK_ANGLE = 48;

        // Podium
        public static final double PODIUM_RPM = 3000;
        public static final double PODIUM_ANGLE = 37;

        // C1
        public static final double C1_RPM = 0;
        public static final double C1_ANGLE = 0;

        // C2
        public static final double C2_RPM = 4100;
        public static final double C2_ANGLE = 28.7d;

        // C3
        public static final double C3_RPM = 4100;
        public static final double C3_ANGLE = 29.5d;

        // Line
        public static final double LINE_RPM = 2000d;
        public static final double LINE_ANGLE = 28.7;

        // 3.5 shhhhhh
        public static final double THREEFIVE_RPM = 500d;
        public static final double THREEFIVE_ANGLE = 27d;

        // Source
        public static final double SOURCE_RPM = -300d; // TODO test
        public static final double SOURCE_ANGLE = 90d; // TODO test

        // Pass
        public static final double NOTE_PASS_ANGLE = 55d;
        public static final double NOTE_PASS_RPM = 4500d;
    }

    public class ClimbConstants {
        public static final boolean CLIMB_RIGHT_MOTOR_INVERT = false;
        public static final boolean CLIMB_LEFT_MOTOR_INVERT = true;
        public static final int CLIMB_MOTOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean CLIMB_MOTOR_BRAKE_MODE = true;
        public static final double UNLOADED_KP = 10;
        public static final double UNLOADED_KI = 0;
        public static final double UNLOADED_KD = 0;
        public static final double LOADED_KP = 0;
        public static final double LOADED_KI = 0;
        public static final double LOADED_KD = 0;
        public static final double GEAR_REDUCTION = 12d;
        public static final double WINCH_DIAMETER_INCHES = 1d;
        public static final double WINCH_CIRCUFERENCE = WINCH_DIAMETER_INCHES * Math.PI;

        public static final double MAX_HEIGHT = 8.83; // In rotations
        public static final double LOWER_LENGTH = 22d; // center of pivot-center of pivot length of lower arm in inches
        public static final double UPPER_LENGTH = 25d; // center of pivot-center of pivot length of upper arm in inches

        public static final double CLIMB_PID_SETPOINT_EXTENDED = MAX_HEIGHT;
        public static final double CLIMB_PID_SETPOINT_RETRACTED = 1;
        public static final double CLIMB_EXTENSION_TOLERANCE = 0;
        public static final double CLIMB_RETRACTION_TOLERANCE = 0.5;
        public static final double CLIMB_RETURN_TO_GROUND_MAX_POWER = 0.05;

        public static final double CLIMB_SYSTEST_POWER = 0.1;
    }

    public class LEDsConstants {
        public static final int LED_LENGTH = 50;

        public static final Map<Integer, Integer> STRAND_START = new HashMap<Integer, Integer>() {
            {
                put(-1, 0);
                put(0, 0);
                put(1, 14);
                put(2, 28);
            }
        };

        public static final Map<Integer, Integer> STRAND_LENGTH = new HashMap<Integer, Integer>() {
            {
                put(-1, LEDsConstants.LED_LENGTH);
                put(0, 14);
                put(1, 15);
                put(2, 22);
            }
        };

        public static final int SWIRL_SEGMENT_SIZE = 5;

        public static final int RED_HUE = 0;
        public static final int ORANGE_HUE = 5;
        public static final int YELLOW_HUE = 15;
        public static final int GREEN_HUE = 240;
        public static final int BLUE_HUE = 120;
        public static final int PURPLE_HUE = 315;
        public static final int PINK_HUE = 355;

        public enum LED_STATES {
            CUSTOMCONTROL(0), 
            DISABLED(1), 
            EMERGENCY(2), 
            START(3), 
            GOOD_POSE(4), 
            COLLECT_PLANNED(5), 
            COLLECTED(6), 
            SHOT(7), 
            FINISHED_CLIMB(8), 
            SHOOTING(9), 
            COLLECTING(10), 
            CHASING(11), 
            CLIMBING(12), 
            BAD_POSE(13), 
            HAS_PIECE(14), 
            HAS_VISION(15), 
            DEFAULT(16), 
            PIVOT_BOTTOM_SWITCH(17), 
            PIVOT_TOP_SWITCH(18), 
            COLLECTOR_BEAMBREAK(19), 
            INDEXER_ENTER_BEAMBREAK(20), 
            INDEXER_EXIT_BEAMBREAK(21);

            private final int priority;

            LED_STATES(int priority) {
                this.priority = priority;
            }

            public int getPriority() {
                return priority;
            }
        }
    }
}


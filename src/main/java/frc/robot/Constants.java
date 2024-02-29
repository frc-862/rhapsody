package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.Swerve;
import frc.thunder.math.InterpolationMap;

public class Constants {

    public static final Path MERCURY_PATH = Paths.get("/home/lvuser/mercury");

    public static final boolean isMercury() {
        return MERCURY_PATH.toFile().exists();
    }

    public static final String HOOT_PATH = "/home/lvuser/signallogger";

    public class DrivetrainConstants { // TODO Get new for new robot
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches * 2; // 2 * x
                                                                                         // distance
                                                                                         // from
                                                                                         // center
                                                                                         // of robot
                                                                                         // to wheel
        public static final double MaxAngularRate = 2 * Math.PI * ( // convert to radians per second
        TunerConstants.kSpeedAt12VoltsMps / Math.PI * Math.sqrt(2 * Math.pow(WHEELBASE, 2))); // free
                                                                                              // speed
                                                                                              // /
                                                                                              // circumference
                                                                                              // of
                                                                                              // circle
                                                                                              // with
                                                                                              // radius
                                                                                              // of
                                                                                              // wheelbase

        public static final double ROT_MULT = 0.015; // TODO Tune for Driver

        public static final double SLOW_ROT_MULT = 0.3; // TODO Tune for Driver
        public static final double SLOW_SPEED_MULT = 0.4; // TODO Tune for Driver

        public static final double SYS_TEST_SPEED_DRIVE = 0.5;
        public static final double SYS_TEST_SPEED_TURN = 1d;

        public static final Translation2d SPEAKER_POSE = new Translation2d(0d, 5.547393);
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

            // TODO check
            public static final int COLLECTOR_MOTOR = 9;
            public static final int INDEXER_MOTOR = 10;
            public static final int PIVOT_ANGLE_MOTOR = 11;
            public static final int FLYWHEEL_MOTOR_BOTTOM = 12;
            public static final int FLYWHEEL_MOTOR_TOP = 13;
            public static final int CLIMB_RIGHT = 14;
            public static final int CLIMB_LEFT = 15;

            // TODO check
            // Cancoders
            public static final int FLYWHEEL_CANCODER = 35;
            public static final int CLIMB_CANCODERR = 36;
            public static final int CLIMB_CANCODERL = 37;
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
            public static final int LED_PORT_1 = 0;
        }
    }

    public static class ControllerConstants {
        public static final int DriverControllerPort = 0;
        public static final int CopilotControllerPort = 1;

        public static final double DEADBAND = 0.1;
    }

    public static class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.0, 0, 0); // TODO:
                                                                                        // Tune
        public static final PIDConstants ROTATION_PID = new PIDConstants(4, 0, 0); // TODO: Tune

        public static final double MAX_MODULE_VELOCITY = Units.feetToMeters(17.3); // f/s to m/s
        public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(19.09); // TODO check

        public static final double CONTROL_LOOP_PERIOD = 0.004; // IS this right?

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2.0, 1, 1.0, 0.5); // TODO get
                                                                                                      // constants

        // For Pathfinding
        // TODO get real poses to pathfind to
        public static final Pose2d TARGET_POSE = new Pose2d(new Translation2d(0, 0), new Rotation2d(0d));

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
        private static final Slot0Configs driveGains = new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0)
                .withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

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

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
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
        private static final double kFrontLeftEncoderOffsetRh = 0.0546875;
        private static final double kFrontLeftXPosInchesRh = 10.825;
        private static final double kFrontLeftYPosInchesRh = 10.825;

        private static final double kFrontRightEncoderOffsetRh = 0.287353515625;
        private static final double kFrontRightXPosInchesRh = 10.825;
        private static final double kFrontRightYPosInchesRh = -10.825;

        private static final double kBackLeftEncoderOffsetRh = 0.243408203125;
        private static final double kBackLeftXPosInchesRh = -10.825;
        private static final double kBackLeftYPosInchesRh = 10.825;

        private static final double kBackRightEncoderOffsetRh = -0.052734375;
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

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                CAN.kFrontLeftSteerMotorId,
                CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId,
                kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches),
                Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                CAN.kFrontRightSteerMotorId,
                CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId,
                kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                CAN.kBackLeftSteerMotorId, CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId,
                kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches),
                Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                CAN.kBackRightSteerMotorId,
                CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId,
                kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches),
                Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

        private static final SwerveModuleConstants FrontLeftRh = ConstantCreator.createModuleConstants(
                CAN.kFrontLeftSteerMotorId,
                CAN.kFrontLeftDriveMotorId, CAN.kFrontLeftEncoderId,
                kFrontLeftEncoderOffsetRh, Units.inchesToMeters(kFrontLeftXPosInchesRh),
                Units.inchesToMeters(kFrontLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants FrontRightRh = ConstantCreator.createModuleConstants(
                CAN.kFrontRightSteerMotorId,
                CAN.kFrontRightDriveMotorId, CAN.kFrontRightEncoderId,
                kFrontRightEncoderOffsetRh, Units.inchesToMeters(kFrontRightXPosInchesRh),
                Units.inchesToMeters(kFrontRightYPosInchesRh), kInvertRightSide);
        private static final SwerveModuleConstants BackLeftRh = ConstantCreator.createModuleConstants(
                CAN.kBackLeftSteerMotorId,
                CAN.kBackLeftDriveMotorId, CAN.kBackLeftEncoderId, kBackLeftEncoderOffsetRh,
                Units.inchesToMeters(kBackLeftXPosInchesRh),
                Units.inchesToMeters(kBackLeftYPosInchesRh), kInvertLeftSide);
        private static final SwerveModuleConstants BackRightRh = ConstantCreator.createModuleConstants(
                CAN.kBackRightSteerMotorId,
                CAN.kBackRightDriveMotorId, CAN.kBackRightEncoderId,
                kBackRightEncoderOffsetRh, Units.inchesToMeters(kBackRightXPosInchesRh),
                Units.inchesToMeters(kBackRightYPosInchesRh), kInvertRightSide);

        public static final Swerve getDrivetrain(Limelights limelights) {
            if (Constants.isMercury()) {
                System.out.println("IS MERCURY");
                return new Swerve(DrivetrainConstants, 250, limelights, FrontLeft, FrontRight,
                        BackLeft, BackRight);
            } else {
                System.out.println("IS RHAPSODY");
                return new Swerve(DrivetrainConstants, 250, limelights, FrontLeftRh, FrontRightRh,
                        BackLeftRh, BackRightRh);
            }
        }
    }

    public class VisionConstants {
        // This is a magic number from gridlock, may need to be changed or removed
        // entirely
        public static final double PROCESS_LATENCY = 0.0472; // TODO test
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0),
                Units.feetToMeters(26.0));
        public static final Translation2d VISION_LIMIT = new Translation2d(Units.feetToMeters(9),
                Units.feetToMeters(5));
        public static final double COLLISION_DEADZONE = 2d;
        public static final double ALIGNMENT_TOLERANCE = 8d; // TODO: make this an actual value
        public static final PIDController TAG_AIM_CONTROLLER = new PIDController(0.1, 0, 0);
        public static final PIDController CHASE_CONTROLLER = new PIDController(0.05, 0, 0);
        public static final int TAG_PIPELINE = 0;
        public static final int SPEAKER_PIPELINE = 1;
        public static final int NOTE_PIPELINE = 2;

        public class Pipelines { // TODO get real
            public static final int APRIL_TAG_3d = 0;
            public static final int APRIL_TAG_2d = 1;
            public static final int CHASE_PIECE = 2; // FOR the collector
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
        public static final String[] SONG_NAMES = { BOH_RHAP_FILEPATH, JEOPARDY_FILEPATH,
                BEWARE_THE_FOREST_MUSHROOMS_FILEPATH, UNDER_PRESSURE_FILEPATH,
                NATIONAL_PARK_FILEPATH, ENCOUNTER_FILEPATH, PIRATES_OF_THE_CARIBBEAN_FILEPATH,
                CRAZY_LITTLE_THING_CALLED_LOVE_FILEPATH, ANOTHER_ONE_BITES_THE_DUST_FILEPATH,
                SWEET_CAROLINE_FILEPATH, WE_ARE_THE_CHAMPIONS_FILEPATH };
        public static final List<String> SET_LIST = Arrays.asList(SONG_NAMES);
    }

    public class CollectorConstants { // TODO: get real
        public static final boolean COLLECTOR_MOTOR_INVERTED = true;
        public static final int COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT = 60; // TODO: make sure they are not set to 0
        public static final boolean COLLECTOR_MOTOR_BRAKE_MODE = false;

        public static final double COLLECTOR_SYSTEST_POWER = 0.25;
    }

    public class FlywheelConstants { // TODO: get real
        public static final boolean MOTOR_TOP_INVERT = true;
        public static final boolean MOTOR_BOTTOM_INVERT = false;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 40;
        public static final boolean MOTOR_BRAKE_MODE = false;
        public static final double MOTOR_KP = 0; // 0.00314;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0;
        public static final double MOTOR_KV = 0.123; // 0.00195;
        public static final double MOTOR_KA = 0;

        public static final double RPM_TOLERANCE = 50d;

        public static final double BIAS_INCREMENT = 1.25; // RPS to bias by per button press
        public static final double COAST_VOLTAGE = 0.1;

        public static final double FLYWHEEL_SYSTEST_POWER = 0.5;
    }

    public class IndexerConstants { // TODO: get real
        public static final boolean MOTOR_INVERT = true;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 60;

        public enum PieceState {
            IN_COLLECT, IN_PIVOT, IN_INDEXER, NONE
        }

        public static final boolean INDEXER_MOTOR_BRAKE_MODE = true;
        public static final double INDEXER_DEFAULT_POWER = 0.3d;
        public static final double INDEXER_MANUAL_POWER = 0.5d;
    }

    public class PivotConstants { // TODO: get real
        public static final boolean MOTOR_INVERT = true; // POS power is up
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean MOTOR_BRAKE_MODE = true;
        public static final double MOTOR_KP = 0;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        public static final double MOTOR_KS = 0;
        public static final double MOTOR_KV = 3;
        public static final double MOTOR_KA = 0;

        public static final double MAGIC_CRUISE_VEL = 0.01; // TODO: get real value
        public static final double MAGIC_ACCEL = 0.02; // TODO: get real value
        public static final double MAGIC_JERK = 0.2; // TODO: get real value

        public static final double ANGLE_TOLERANCE = 0.5d;

        public static final double ENCODER_OFFSET = 0.6118; // In rotations
        public static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;
        public static final double ROTOR_TO_ENCODER_RATIO = 618.75;

        public static final double BIAS_INCREMENT = 1d; // Degrees to bias by per button press TODO get amount to bias by

        public static final double STOW_ANGLE = 35d;

        public static final double MAX_INDEX_ANGLE = 40d;

        public static final double MIN_ANGLE = 25d;
        public static final double MAX_ANGLE = 105d;
    }

    public class ShooterConstants {

        public static final double FAR_WING_X = 3.3;

        // Distance in meters, angle in degrees
        public static final InterpolationMap ANGLE_MAP = new InterpolationMap() {
            {
                // As distance gets smaller angle goes up
                put(1.08d, 60d);
                put(1.97d, 47d);
                put(2.89d, 39d);
                put(3.796, 37d);
                put(4.72, 34d);
                put(5.74, 27d);

            }
        };

        // Distance in meters, speed in RPM
        public static final InterpolationMap SPEED_MAP = new InterpolationMap() {
            {
                // As distance get smaller RPM gets smaller
                put(1.08d, 2000d);
                put(1.97d, 2250d);
                put(2.89d, 2500d);
                put(3.796, 2500d);
                put(4.72, 2750d);
                put(5.74, 3800d);
            }
        };

        public enum ShootingState {
            AIM, SHOOT, SHOT
        }
    }

    public class CandConstants { // TODO get real
        // Amp
        public static final double AMP_TOP_RPM = 300; // 250?
        public static final double AMP_BOTTOM_RPM = 450;
        public static final double AMP_ANGLE = 103.5;

        // PointBlank
        public static final double POINT_BLANK_RPM = 2000;
        public static final double POINT_BLANK_ANGLE = 60;

        // Podium
        public static final double PODIUM_RPM = 3000;
        public static final double PODIUM_ANGLE = 45;

        // C1
        public static final double C1_RPM = 0;
        public static final double C1_ANGLE = 0;

        // C2
        public static final double C2_RPM = 0;
        public static final double C2_ANGLE = 0;

        // C3
        public static final double C3_RPM = 0;
        public static final double C3_ANGLE = 0;

        // Line
        public static final double LINE_RPM = 0;
        public static final double LINE_ANGLE = 0;

        // Source
        public static final double SOURCE_RPM = -300d; // TODO test
        public static final double SOURCE_ANGLE = 45d; // TODO test

        // TODO find time to shoot
        public static final double TIME_TO_SHOOT = 2d; // Time in seconds it takes from indexer start to flywheel exit
    }

    public class ClimbConstants { // TODO: find real values
        public static final boolean CLIMB_RIGHT_MOTOR_INVERT = false;
        public static final boolean CLIMB_LEFT_MOTOR_INVERT = false;
        public static final int CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int CLIMB_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final boolean CLIMB_MOTOR_BRAKE_MODE = true;
        public static final double EXTEND_KP = 0;
        public static final double EXTEND_KI = 0;
        public static final double EXTEND_KD = 0;
        public static final double RETRACT_KP = 0;
        public static final double RETRACT_KI = 0;
        public static final double RETRACT_KD = 0;
        public static final double GEAR_REDUCTION = 20d;
        public static final double WINCH_DIAMETER_INCHES = 1d;
        public static final double WINCH_CIRCUFERENCE = WINCH_DIAMETER_INCHES * Math.PI;

        public static final double MAX_HEIGHT = 999d;
        public static final double LOWER_LENGTH = 22d; // center of pivot-center of pivot length of lower arm in inches
        public static final double UPPER_LENGTH = 25d; // center of pivot-center of pivot length of upper arm in inches

        public static final Pose3d LOWER_OFFSET = new Pose3d(); // NOTE: Poses are in meters despite george washington's
                                                                // best efforts
        public static final Pose3d UPPER_OFFSET = new Pose3d(); // NOTE 2: these poses should exclude side to side
                                                                // offset, since it gets set below

        public static final Transform3d LEFT_RIGHT_OFFSET = new Transform3d(); // NOTE 3: this is the side to side
                                                                               // offset of the pivot point of the arms,
                                                                               // should exclude anything but side to
                                                                               // side values
        public static final double CLIMB_PID_SETPOINT_EXTENDED = 10; // TODO: find real values
        public static final double CLIMB_PID_SETPOINT_RETRACTED = 0;
        public static final double CLIMB_EXTENSION_TOLERANCE = 0;
        public static final double CLIMB_RETRACTION_TOLERANCE = 0;
        public static final double CLIMB_RETURN_TO_GROUND_MAX_POWER = 0.05;

        public static final double CLIMB_SYSTEST_POWER = 0.1;

        public enum CLIMBER_STATES {
            CLIMBED, GROUNDED, STOW
        }
    }

    public class LEDsConstants {
        public static final int LED_LENGTH = 14;

        public static final int SWIRL_SEGMENT_SIZE = 5;

        public static final int RED_HUE = 0;
        public static final int ORANGE_HUE = 5;
        public static final int YELLOW_HUE = 15;
        public static final int GREEN_HUE = 240;
        public static final int BLUE_HUE = 120;
        public static final int PURPLE_HUE = 315;
        public static final int PINK_HUE = 355;

        public enum LED_STATES {
            DISABLED(0),
            EMERGENCY(1),
            START(2),
            COLLECTED(3),
            SHOT(4),
            FINISHED_CLIMB(5),
            SHOOTING(6),
            COLLECTING(7),
            CHASING(8),
            CLIMBING(9),
            HAS_PIECE(10),
            HAS_VISION(11),
            DEFAULT(12);

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

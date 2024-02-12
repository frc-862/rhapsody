package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
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
import frc.thunder.shuffleboard.LightningShuffleboard;

/** Add your docs here. */
public class Constants {

    public static final Path MERCURY_PATH = Paths.get("/home/lvuser/mercury");

    public static final boolean isMercury() {
        return MERCURY_PATH.toFile().exists();
    }

    public static final String HOOT_PATH = "/home/lvuser/signallogger";

    public class DrivetrainConstants { // TODO Get new for new robot
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        private static final double WHEELBASE = TunerConstants.kFrontLeftXPosInches * 2; // 2 * x distance from center of robot to wheel
        public static final double MaxAngularRate = 2 * Math.PI * ( // convert to radians per second
        TunerConstants.kSpeedAt12VoltsMps / Math.PI * Math.sqrt(2 * Math.pow(WHEELBASE, 2))); //free speed / circumference of circle with radius of wheelbase

        public static final double ROT_MULT = 0.015; // TODO Tune for Driver

        public static final double SLOW_ROT_MULT = 0.007; // TODO Tune for Driver
        public static final double SLOW_SPEED_MULT = 0.4; // TODO Tune for Driver

        public static final double SYS_TEST_SPEED_DRIVE = 0.5;
        public static final double SYS_TEST_SPEED_TURN = 1d;
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
            public static final int INDEXER_MOTOR = 11;
            public static final int PIVOT_ANGLE_MOTOR = 12;
            public static final int FLYWHEEL_MOTOR_TOP = 13;
            public static final int FLYWHEEL_MOTOR_BOTTOM = 14;
            public static final int CLIMB_RIGHT = 15;
            public static final int CLIMB_LEFT = 16;

            // TODO check
            // Cancoders
            public static final int FLYWHEEL_CANCODER = 35;
            public static final int CLIMB_CANCODERR = 36;
            public static final int CLIMB_CANCODERL = 37;
            public static final int PIVOT_ANGLE_CANCODER = 38;

            public static final String CANBUS_FD = "Canivore";
            public static final String RIO_CANBUS = "rio";
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
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.0, 0, 0); // TODO: Tune
        public static final PIDConstants ROTATION_PID = new PIDConstants(4, 0, 0); // TODO: Tune

        public static final double MAX_MODULE_VELOCITY = Units.feetToMeters(17.3); // f/s to m/s
        public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(19.09); // TODO check

        public static final double CONTROL_LOOP_PERIOD = 0.004; // IS this right?

        //For Pathfinding
        public static final Pose2d TARGET_POSE = new Pose2d(new Translation2d(-1, 2), new Rotation2d(0d));
        public static final double GOAL_END_VELOCITY = 0; // Goal end velocity in meters/sec
        public static final double ROTATION_DELAY_DISTACE = 0d; // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2.0, 1, 1.0, 0.5); //TODO get constants
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

        public static final double kDriveGearRatio = 6.122448979591837;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kWheelRadiusInches = 2;

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
        private static final double kFrontLeftEncoderOffsetRh = -0.03564453125;
        private static final double kFrontLeftXPosInchesRh = 13.5;
        private static final double kFrontLeftYPosInchesRh = 13.5;

        private static final double kFrontRightEncoderOffsetRh = 0.2978515625;
        private static final double kFrontRightXPosInchesRh = 13.5;
        private static final double kFrontRightYPosInchesRh = -13.5;

        private static final double kBackLeftEncoderOffsetRh = 0.245361328125;
        private static final double kBackLeftXPosInchesRh = -13.5;
        private static final double kBackLeftYPosInchesRh = 13.5;

        private static final double kBackRightEncoderOffsetRh = 0.044189453125;
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


        public static final Swerve getDrivetrain(Limelights limelights) {
            if (Constants.isMercury()) {
                System.out.println("IS MERCURY");
                return new Swerve(DrivetrainConstants, 250, limelights, FrontLeft, FrontRight, BackLeft,
                        BackRight);
            } else {
                System.out.println("IS RHAPSODY");
                return new Swerve(DrivetrainConstants, 250, limelights, FrontLeftRh, FrontRightRh, BackLeftRh,
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
        public static final double TIP_DEADZONE = 2d;
        public static final double ALIGNMENT_TOLERANCE = 4d; // TODO: make this an actual value
        public static final PIDController TAG_AIM_CONTROLLER = new PIDController(0.05, 0, 0);
        public static final PIDController CHASE_CONTROLLER = new PIDController(0.05, 0, 0);
        public static final int TAG_PIPELINE = 0;
        public static final int NOTE_PIPELINE = 2;

        public static final double ACCELERATION_DUE_TO_GRAVITY = 9.80665;
        public static final double COLLISION_ACCELERATION_TOLERANCE_PERCENTAGE = 0.25; //percent of motorAcceleration
        public static final double DISTANCE_FROM_CENTER_TO_MODULE = Math.hypot(13.5, 13.5);
        public static final double DISTANCE_FROM_CENTER_TO_PIGEON = 0d;

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
        public static final String[] SONG_NAMES = {
            BOH_RHAP_FILEPATH, JEOPARDY_FILEPATH, BEWARE_THE_FOREST_MUSHROOMS_FILEPATH, 
                UNDER_PRESSURE_FILEPATH, NATIONAL_PARK_FILEPATH, ENCOUNTER_FILEPATH, PIRATES_OF_THE_CARIBBEAN_FILEPATH, 
                CRAZY_LITTLE_THING_CALLED_LOVE_FILEPATH, ANOTHER_ONE_BITES_THE_DUST_FILEPATH, SWEET_CAROLINE_FILEPATH, WE_ARE_THE_CHAMPIONS_FILEPATH};
        public static final List<String> SET_LIST = Arrays.asList(SONG_NAMES);
    }

    public class CollectorConstants { // TODO: get real
        public static final boolean COLLECTOR_MOTOR_INVERTED = false; // TODO check once collector installed
        public static final int COLLECTOR_MOTOR_SUPPLY_CURRENT_LIMIT = 0; // TODO: make sure they are not set to 0
        public static final int COLLECTOR_MOTOR_STATOR_CURRENT_LIMIT = 0; // TODO: make sure they are not set to 0
        public static final boolean COLLECTOR_MOTOR_BRAKE_MODE = false;
    }

    public class FlywheelConstants { // TODO: get real
        public static final boolean FLYWHEEL_MOTOR_TOP_INVERT = false;
        public static final boolean FLYWHEEL_MOTOR_BOTTOM_INVERT = false;
        public static final int FLYWHEEL_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int FLYWHEEL_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final boolean FLYWHEEL_MOTOR_BRAKE_MODE = false;
        public static final double FLYWHEEL_MOTOR_KP = 0;
        public static final double FLYWHEEL_MOTOR_KI = 0;
        public static final double FLYWHEEL_MOTOR_KD = 0;
        public static final double FLYWHEEL_MOTOR_KS = 0;
        public static final double FLYWHEEL_MOTOR_KV = 0;

        public static final double RPM_TOLERANCE = 0;

        public static final double BIAS_INCREMENT = 0; // RPM to bias by per button press TODO get amount to bias by
        public static final double COAST_VOLTAGE = 0.1;
    }

    public class IndexerConstants { // TODO: get real
        public static final boolean INDEXER_MOTOR_INVERTED = false;
        public static final int INDEXER_MOTOR_SUPPLY_CURRENT_LIMIT = 0;
        public static final int INDEXER_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final boolean INDEXER_MOTOR_BRAKE_MODE = true;
        public static final double INDEXER_DEFAULT_POWER = 0.3;
    }

    public class PivotConstants { // TODO: get real
        public static final boolean PIVOT_MOTOR_INVERT = false;
        public static final int PIVOT_MOTOR_STATOR_CURRENT_LIMIT = 0;
        public static final boolean PIVOT_MOTOR_BRAKE_MODE = true;
        public static final double PIVOT_MOTOR_KP = 0;
        public static final double PIVOT_MOTOR_KI = 0;
        public static final double PIVOT_MOTOR_KD = 0;
        public static final double PIVOT_MOTOR_KS = 0;
        public static final double PIVOT_MOTOR_KV = 0;

        public static final double ANGLE_TOLERANCE = 0;

        public static final double ENCODER_OFFSET = 0d;
        public static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;
        public static final double ENCODER_TO_ROTOR_RATIO = 1d;

        public static final double BIAS_INCREMENT = 1d; // Degrees to bias by per button press TODO get amount to bias by
    }

    public class ShooterConstants {
        public static final double STOW_ANGLE = 0;

        public static final double FAR_WING_X = 3.3;

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

    public class CandConstants {
        //Amp
        public static final double AMP_TOP_RPM = 0;
        public static final double AMP_BOTTOM_RPM = 0;
        public static final double AMP_ANGLE = 0;

        //PointBlank
        public static final double POINT_BLANK_RPM = 0;
        public static final double POINT_BLANK_ANGLE = 0;

        //Podium
        public static final double PODIUM_RPM = 0;
        public static final double PODIUM_ANGLE = 0;
    }

    public class ClimbConstants { //TODO: find real values
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
        public static final double LOWER_LENGTH = 22d; //center of pivot-center of pivot length of lower arm in inches
        public static final double UPPER_LENGTH = 25d; //center of pivot-center of pivot length of upper arm in inches

        public static final Pose3d LOWER_OFFSET = new Pose3d(); //NOTE: Poses are in meters despite george washington's best efforts
        public static final Pose3d UPPER_OFFSET = new Pose3d(); //NOTE 2: these poses should exclude side to side offset, since it gets set below

        public static final Transform3d LEFT_RIGHT_OFFSET = new Transform3d(); //NOTE 3: this is the side to side offset of the pivot point of the arms, should exclude anything but side to side values
        public static final double CLIMB_PID_SETPOINT_EXTENDED = 10; //TODO: find real values
        public static final double CLIMB_PID_SETPOINT_RETRACTED = 0;
        public static final double CLIMB_EXTENSION_TOLERANCE = 0;
        public static final double CLIMB_RETRACTION_TOLERANCE = 0;
        public static final double CLIMB_RETURN_TO_GROUND_MAX_POWER = 0.05;

        public static final double CLIMB_TEST_POWER = .1;

        public enum CLIMBER_STATES{
            CLIMBED, GROUNDED, STOW
        }

    }

    public class LEDsConstants {
        public static final int LED_PWM_PORT = 0;
        public static final int LED_LENGTH = 14;

        public static final int SWRIL_SEGMENT_SIZE = 5;
        
        public static final int RED_HUE = 0;
        public static final int ORANGE_HUE = 5;
        public static final int YELLOW_HUE = 15;
        public static final int GREEN_HUE = 240;
        public static final int BLUE_HUE = 120;
        public static final int PURPLE_HUE = 315;
        public static final int PINK_HUE = 355;

        public enum LED_STATES{
            DISABLED(0),
            MIXER(1),
            COLLECTED(2), 
            SHOT(3), 
            FINISHED_CLIMB(4), 
            SHOOTING(5), 
            COLLECTING(6),
            CHASING(7), 
            CLIMBING(8),
            HAS_PIECE(9), 
            HAS_VISION(10),
            OFF(11);

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
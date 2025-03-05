package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final String CAN_BUS_NAME = "*";
    public static final double STICK_DEADBAND = 0.1;

    public static final class Swerve {
        public static final int PIGEON_ID = 0;

        public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4n
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.Inches.of(21.75).in(Meters);
        public static final double WHEEL_BASE = Units.Inches.of(21.75).in(Meters);
        public static final double WHEEL_CIRCUMFRENCE = CHOSEN_MODULE.wheelCircumference;
        public static final double WHEEL_RADIUS = CHOSEN_MODULE.wheelDiameter / 2;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 5.8;
        public static final LinearVelocity MAX_SPEED_UNITS = Units.MetersPerSecond.of(5.8);
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);
        /**
         * <p>
         * Pose estimator standard deviation for encoder & gyro data
         * </p>
         * <b>Units:</b> Meters
         */
        public static final double MEASUREMENT_STD_DEVS_POS = 0.05;

        /**
         * <p>
         * Pose estimator standard deviation for encoder & gyro data
         * </p>
         * <b>Units:</b> Radians
         */
        public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(5, Units.Degrees);

        public static class TELEOP_AUTO_ALIGN {
            // TODO: Test if this actually works LOL
            public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED = Units.MetersPerSecond
                    .of(MAX_SPEED_UNITS.in(MetersPerSecond) / 4);

            public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(10);
            public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(1);
            public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(5);
            public static final LinearVelocity MIN_DRIVER_OVERRIDE = Swerve.MAX_SPEED_UNITS.div(10);

            public static final PIDController TRANS_CONTROLLER = new PIDController(
                    4,
                    0,
                    0);
            public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

            public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
                    3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
                            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
            public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

            public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(1);

            static {
                TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

                ROTATION_CONTROLLER.enableContinuousInput(0, 360);
                ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
            }

            public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
                    TRANS_CONTROLLER,
                    TRANS_CONTROLLER,
                    ROTATION_CONTROLLER);
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.100342);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.081543);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.374268);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.068115);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static class constField {
        public static Optional<Alliance> ALLIANCE = Optional.empty();
        public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
        public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

        /**
         * Boolean that controls when the path will be mirrored for the red
         * alliance. This will flip the path being followed to the red side of the
         * field.
         * The origin will remain on the Blue side.
         * 
         * @return If we are currently on Red alliance. Will return false if no alliance
         *         is found
         */
        public static boolean isRedAlliance() {
            var alliance = ALLIANCE;
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        /*
         * All poses on the field, defined by their location on the BLUE Alliance
         */
        public static class POSES {
            public static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());
            public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

            // BRANCH POSES
            public static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.fromDegrees(0));
            public static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.fromDegrees(0));
            public static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
            public static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
            public static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
            public static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
            public static final Pose2d REEF_G = new Pose2d(5.805, 3.863, Rotation2d.fromDegrees(180));
            public static final Pose2d REEF_H = new Pose2d(5.805, 4.189, Rotation2d.fromDegrees(180));
            public static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
            public static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
            public static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
            public static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));

            // CORAL STATION POSES
            public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-54.5));
            public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-54.5));
            public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
            public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

            // processor poses
            public static final Pose2d PROCESSOR = new Pose2d(6, .77, Rotation2d.fromDegrees(-90));

            private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
                    REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
            private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

            private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D,
                    REEF_E,
                    REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

            private static final Pose2d[] RED_POSES = getRedAlliancePoses();

            private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
                    LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
            private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

            private static final List<Pose2d> BLUE_PROCESSOR_POSE = List.of(PROCESSOR);
            private static final List<Pose2d> RED_PROCESSOR_POSE = getRedProcessorPoses();

        }

        public static Pose2d getRedAlliancePose(Pose2d bluePose) {
            return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
                    FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
                    bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
        }

        private static Pose2d[] getRedAlliancePoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

            for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
            }
            return returnedPoses;
        }

        private static List<Pose2d> getRedReefPoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

            for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
            }

            return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
                    returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9],
                    returnedPoses[10],
                    returnedPoses[11]);
        }

        private static List<Pose2d> getRedCoralStationPoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

            for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
            }

            return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
        }

        private static List<Pose2d> getRedProcessorPoses() {
            Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_PROCESSOR_POSE.size()];

            for (int i = 0; i < POSES.BLUE_PROCESSOR_POSE.size(); i++) {
                returnedPoses[i] = getRedAlliancePose(POSES.BLUE_PROCESSOR_POSE.get(i));
            }

            return List.of(returnedPoses[0]);
        }

        /**
         * Gets the positions of all of the necessary field elements on the field. All
         * coordinates are in meters and are relative to the blue alliance.
         * 
         * @see <a href=
         *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
         *      Robot Coordinate Systems</a>
         * @return An array of field element positions
         */
        public static Supplier<Pose2d[]> getFieldPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_POSES;

            }
            return () -> POSES.BLUE_POSES;
        }

        /**
         * Gets the positions of all of the necessary field elements on the field. All
         * coordinates are in meters and are relative to the blue alliance.
         * 
         * @see <a href=
         *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
         *      Robot Coordinate Systems</a>
         * @return An array of the reef branches for your alliance
         */
        public static Supplier<List<Pose2d>> getReefPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_REEF_POSES;

            }
            return () -> POSES.BLUE_REEF_POSES;
        }

        public static Supplier<List<Pose2d>> getCoralStationPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_CORAL_STATION_POSES;
            }
            return () -> POSES.BLUE_CORAL_STATION_POSES;
        }

        public static Supplier<List<Pose2d>> getProcessorPositions() {
            if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
                return () -> POSES.RED_PROCESSOR_POSE;
            }
            return () -> POSES.BLUE_PROCESSOR_POSE;
        }

    }

    /* Vision Constants */
    public static class constVision {
        public static final String[] LIMELIGHT_NAMES = new String[] { "limelight-front", "limelight-back" };

        /**
         * <p>
         * Pose estimator standard deviation for vision data
         * <p>
         * <b>Units:</b> Meters
         */
        public static final double MEGA_TAG2_STD_DEVS_POSITION = 0.7;

        /**
         * <p>
         * Pose estimator standard deviation for vision data
         * </p>
         * <b>Units:</b> Radians
         */
        public static final double MEGA_TAG2_STD_DEVS_HEADING = 9999999;

        /**
         * <p>
         * Pose estimator standard deviation for vision data
         * </p>
         * <b>Units:</b> Meters
         */
        public static final double MEGA_TAG1_STD_DEVS_POSITION = .3;

        public static final double MEGA_TAG1_STD_DEVS_HEADING = .1;
        /**
         * <p>
         * Maximum rate of rotation before we begin rejecting pose updates
         * </p>
         */
        public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

        /**
         * The area that one tag (if its the only tag in the update) needs to exceed
         * before being accepted
         */
        public static final double AREA_THRESHOLD = 0.2;

        // The below values are accounted for in the limelight interface, NOT in code
        public static class LIMELIGHT_FRONT {
            public static final Distance LL_FORWARD = Units.Meters.of(0.2921);
            public static final Distance LL_RIGHT = Units.Meters.of(-0.274);
            public static final Distance LL_UP = Units.Meters.of(0.2032);

            public static final Angle LL_ROLL = Units.Degrees.of(180);
            public static final Angle LL_PITCH = Units.Degrees.of(15);
            public static final Angle LL_YAW = Units.Degrees.of(-40);
        }
    }

    /* Intake Constants */
    public static final class constIntake {
        public static TalonFXSConfiguration INTAKE_CONFIG = new TalonFXSConfiguration();
        static {
            INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            INTAKE_CONFIG.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

            INTAKE_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            INTAKE_CONFIG.Slot0.kS = 1; // Volts to overcome static friction
            INTAKE_CONFIG.Slot0.kG = 0;
            INTAKE_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
            INTAKE_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
            INTAKE_CONFIG.Slot0.kP = 0.1;
            INTAKE_CONFIG.Slot0.kI = 0;
            INTAKE_CONFIG.Slot0.kD = 0;

            // Inches the outside of the wheel has moved
            INTAKE_CONFIG.ExternalFeedback.SensorToMechanismRatio = 0.31847;
            INTAKE_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 400;
            INTAKE_CONFIG.MotionMagic.MotionMagicAcceleration = 3000;
        }
        public static final int MOTOR_ID = 17;
        public static final double OUTTAKE_VOLTAGE = 3;
        public static final double INTAKE_VOLTAGE = -3;
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            // ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 140;
            // ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 65;
            // ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
            // ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(72).in(Units.Inches);
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
                    .in(Units.Inches);

            ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            // Elevator motors will provide feedback in INCHES the carriage has moved
            ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.343; // 17 inches to ground

            ELEVATOR_CONFIG.Slot0.kG = 0.75; // Volts to overcome gravity
            ELEVATOR_CONFIG.Slot0.kS = 0.5; // Volts to overcome static friction
            ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
            ELEVATOR_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
            ELEVATOR_CONFIG.Slot0.kP = 1.3;
            ELEVATOR_CONFIG.Slot0.kI = 0.001;
            ELEVATOR_CONFIG.Slot0.kD = 0.1;

            ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 400;
            ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 500;
        }
        public static TalonFXConfiguration COAST_MODE_CONFIGURATION = new TalonFXConfiguration();
        static {
            COAST_MODE_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            COAST_MODE_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        public static final Distance DEADZONE_DISTANCE = Units.Inches.of(2);
        public static final double MULTIPLIER_DEADZONE = 0.1;
        public static final int LEFT_MOTOR_FOLLOWER_ID = 13;
        public static final int RIGHT_MOTOR_LEADER_ID = 14;

        public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
        public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

        /**
         * The voltage supplied to the motor in order to zero
         */
        public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);
        /**
         * The value that the motor reports when it is at it's zeroed position. This
         * may not necessarily be 0 due to mechanical slop
         */
        public static final Distance ZEROED_POS = Units.Meters.of(0);

        /**
         * The velocity that the motor goes at once it has zeroed (and can no longer
         * continue in that direction)
         */
        public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

        /**
         * The elapsed time required to consider the motor as zeroed
         */
        public static final Time ZEROED_TIME = Units.Seconds.of(1);
        public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);
    }

    public enum reefPosition {
        NONE,
        L1,
        L2,
        L3,
        L4
    }

    public static final class Limelight {
        // public static final double limelightHeight = Units.inchesToMeters(20.0);
        // public static final double limelightAngle = Units.degreesToRadians(30.0);
        public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    }

    public static class AUTO {
        // This PID is implemented on the Drivetrain subsystem
        public static final double AUTO_DRIVE_P = 9;
        public static final double AUTO_DRIVE_I = 0;
        public static final double AUTO_DRIVE_D = 0;
        public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(Constants.AUTO.AUTO_DRIVE_P,
                Constants.AUTO.AUTO_DRIVE_I,
                Constants.AUTO.AUTO_DRIVE_D);

        public static final double AUTO_STEER_P = 5.6; // 5.7 is also pretty good if we begin seeing undershooting
        public static final double AUTO_STEER_I = 0.0;
        public static final double AUTO_STEER_D = 0.0;
        public static final PIDConstants AUTO_STEER_PID = new PIDConstants(Constants.AUTO.AUTO_STEER_P,
                Constants.AUTO.AUTO_STEER_I,
                Constants.AUTO.AUTO_STEER_D);

        public static final Mass MASS = Units.Kilograms.of(20);
        // TODO: Calculate the real vaule
        public static final double MOI = 6.0;
        public static final double WHEEL_COF = 1.2;
        public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1).withReduction(5.36);
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(2, 5.5, WHEEL_COF,
                DRIVE_MOTOR,
                Swerve.DRIVE_CURRENT_LIMIT, 1);

        public static final Translation2d[] MODULE_OFFSETS = {
                new Translation2d(Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0) };

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS.in(Kilograms), MOI, MODULE_CONFIG,
                MODULE_OFFSETS);
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constField;
import frc.robot.Constants.constVision;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public static boolean gyroCheck;
    public static boolean ampGyroCheck;
    public static int ampMultiplierCheck;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public Pose2d mt2Pose;
    public double timeFromLastUpdate = 0;
    public double lastSimTime = Timer.getFPGATimestamp();
    Pose2d desiredAlignmentPose = Pose2d.kZero;
    RobotConfig config;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID, Constants.CAN_BUS_NAME);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        // The absolute encoders need time to initialize
        Timer.delay(2.5);
        // if (!constField.isRedAlliance()) {
        // gyro.setYaw(180);
        // } else {
        // gyro.setYaw(0);
        // }
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
                new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
                new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS),
                new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
        // getGyroYaw(), getModulePositions());

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                getModulePositions(),
                new Pose2d(), VecBuilder.fill(
                        Constants.Swerve.MEASUREMENT_STD_DEVS_POS,
                        Constants.Swerve.MEASUREMENT_STD_DEVS_POS,
                        Constants.Swerve.MEASUREMENT_STD_DEV_HEADING),
                VecBuilder.fill(
                        constVision.MEGA_TAG2_STD_DEVS_POSITION,
                        constVision.MEGA_TAG2_STD_DEVS_POSITION,
                        constVision.MEGA_TAG2_STD_DEVS_HEADING));

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        // Configure AutoBuilder last
        AutoBuilder.configure(this::getPoseEstimator, this::resetOdometry, this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(Constants.AUTO.AUTO_DRIVE_PID, Constants.AUTO.AUTO_STEER_PID), config,
                () -> constField.isRedAlliance(), this);

    }

    public AngularVelocity getVelocityToRotate(Rotation2d desiredYaw) {
        double yawSetpoint = Constants.Swerve.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.getThetaController()
                .calculate(getRotation().getRadians(), desiredYaw.getRadians());

        // limit the PID output to our maximum rotational speed
        yawSetpoint = MathUtil.clamp(yawSetpoint, -Constants.Swerve.TURN_SPEED.in(Units.RadiansPerSecond),
                Constants.Swerve.TURN_SPEED.in(Units.RadiansPerSecond));

        return Units.RadiansPerSecond.of(yawSetpoint);
    }

    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> limelightPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/Drivetrain/Limelight Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> desiredAlignmentPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/Drivetrain/Desired Alignment Pose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SmartDashboard/Drivetrain/Desired States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> actualStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SmartDashboard/Drivetrain/Actual States", SwerveModuleState.struct).publish();

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
                mSwerveMods[0].getState(),
                mSwerveMods[1].getState(),
                mSwerveMods[2].getState(),
                mSwerveMods[3].getState());
    }

    // public void getLimelightPose() {
    // LimelightHelpers.SetRobotOrientation("limelight-front",
    // getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0,
    // 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    // if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if
    // our angular velocity is greater
    // // than 720 degrees per second, ignore
    // // vision updates
    // {
    // doRejectUpdate = true;
    // }
    // if (mt2.tagCount == 0) {
    // doRejectUpdate = true;
    // }
    // if (!doRejectUpdate) {
    // swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7,
    // 9999999));
    // swervePoseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    // mt2Pose = mt2.pose;
    // }
    // }

    @SuppressWarnings("removal")
    public double getGyroRate() {
        return gyro.getRate();
    }

    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
    }

    public void updatePoseEstimator() {
        swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeChassisSpeeds, timeFromLastUpdate));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void autoDrive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] desiredModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Rotation2d getRotation() {
        return swervePoseEstimator.getEstimatedPosition().getRotation();
    }

    public boolean isAtRotation(Rotation2d desiredRotation) {
        return (getRotation().getMeasure()
                .compareTo(desiredRotation.getMeasure()
                        .minus(Constants.Swerve.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0)
                &&
                getRotation().getMeasure()
                        .compareTo(desiredRotation.getMeasure()
                                .plus(Constants.Swerve.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
    }

    public boolean isAtPosition(Pose2d desiredPose2d) {
        return Units.Meters
                .of(getPoseEstimator().getTranslation().getDistance(desiredPose2d.getTranslation()))
                .lte(Constants.Swerve.TELEOP_AUTO_ALIGN.AT_POINT_TOLERANCE);
    }

    public boolean atPose(Pose2d desiredPose) {
        return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
    }

    /**
     * Calculate the ChassisSpeeds needed to align the robot to the desired pose.
     * This must be called every loop until you reach the desired pose.
     * 
     * @param desiredPose The desired pose to align to
     * @return The ChassisSpeeds needed to align the robot to the desired pose
     */
    public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose) {
        desiredAlignmentPose = desiredPose;
        // TODO: This might run better if instead of 0, we use
        // Constants.Swerve.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond);.
        // I dont know why. it might though
        return Constants.Swerve.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(getPoseEstimator(),
                desiredPose, 0,
                desiredPose.getRotation());
    }

    /**
     * Returns the closest reef branch to the robot.
     * 
     * @param leftBranchRequested If we are requesting to align to the left or right
     *                            branch
     * @return The desired reef branch face to align to
     */
    public Pose2d getDesiredReef(boolean leftBranchRequested) {
        // Get the closest reef branch face using either branch on the face
        List<Pose2d> reefPoses = constField.getReefPositions().get();
        Pose2d currentPose = getPoseEstimator();
        Pose2d desiredReef = currentPose.nearest(reefPoses);
        int closestReefIndex = reefPoses.indexOf(desiredReef);

        // Invert faces on the back of the reef so they're always relative to the driver
        if (closestReefIndex > 3 && closestReefIndex < 10) {
            leftBranchRequested = !leftBranchRequested;
        }

        // If we were closer to the left branch but selected the right branch (or
        // vice-versa), switch to our desired branch
        if (leftBranchRequested && (closestReefIndex % 2 == 1)) {
            desiredReef = reefPoses.get(closestReefIndex - 1);
        } else if (!leftBranchRequested && (closestReefIndex % 2 == 0)) {
            desiredReef = reefPoses.get(closestReefIndex + 1);
        }
        return desiredReef;
    }

    public AngularVelocity getVelocityToRotate(Angle desiredYaw) {
        return getVelocityToRotate(Rotation2d.fromDegrees(desiredYaw.in(Units.Degrees)));
    }

    public Boolean isAligned() {
        return desiredAlignmentPose.getTranslation().getDistance(getPoseEstimator()
                .getTranslation()) <= Constants.Swerve.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_TOLERANCE.in(Units.Meters) && Math.abs(desiredAlignmentPose.getRotation().getDegrees() - (getPoseEstimator()
                .getRotation().getDegrees())) <= Constants.Swerve.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE.in(Units.Degrees);
    }

    public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
            LinearVelocity xVelocity,
            LinearVelocity yVelocity,
            AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop, Distance maxAutoDriveDistance) {
        desiredAlignmentPose = desiredTarget;
        int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
        LimelightHelpers.setLEDMode_ForceOn(Constants.constVision.LIMELIGHT_NAMES[0]);
        LimelightHelpers.setLEDMode_ForceOn(Constants.constVision.LIMELIGHT_NAMES[1]);

        if (distanceFromTarget.gte(maxAutoDriveDistance)) {
            // Rotational-only auto-align
            drive(new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond),
                    yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)),
                    getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), true, isOpenLoop);
        } else {
            // Full auto-align
            ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget);

            // Speed limit based on elevator height
            LinearVelocity linearSpeedLimit = Constants.Swerve.MAX_SPEED_UNITS.times(elevatorMultiplier);
            AngularVelocity angularSpeedLimit = Constants.Swerve.TURN_SPEED.times(elevatorMultiplier);

            if ((desiredChassisSpeeds.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
                    || (desiredChassisSpeeds.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
                    || (desiredChassisSpeeds.omegaRadiansPerSecond > angularSpeedLimit.in(Units.RadiansPerSecond))) {

                desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vxMetersPerSecond, 0,
                        linearSpeedLimit.in(MetersPerSecond));
                desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vyMetersPerSecond, 0,
                        linearSpeedLimit.in(MetersPerSecond));
                desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredChassisSpeeds.omegaRadiansPerSecond,
                        0,
                        angularSpeedLimit.in(RadiansPerSecond));
            }

            // drive(desiredChassisSpeeds, isOpenLoop);
            autoDrive(desiredChassisSpeeds, isOpenLoop);
            SmartDashboard.putNumber("AutoChassisSpeeds", desiredChassisSpeeds.omegaRadiansPerSecond);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPoseEstimator() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    // public void setPose(Pose2d pose) {
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    // }

    public void resetOdometry(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);

    }

    public Rotation2d getHeading() {
        return getPoseEstimator().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPoseEstimator().getTranslation(), heading));
    }

    // public void zeroHeading() {
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
    // new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // swervePoseEstimator.resetRotation(getHeading());
    // }

    public Rotation2d getGyroYaw() {
        double yaw = gyro.getYaw().getValueAsDouble() % 360;
        return (yaw < 0) ? Rotation2d.fromDegrees(yaw + 360) : Rotation2d.fromDegrees(yaw);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void updateTimer() {
        timeFromLastUpdate = Timer.getFPGATimestamp() - lastSimTime;
        lastSimTime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        updateTimer();
        updatePoseEstimator();
        limelightPosePublisher.set(getPoseEstimator());
        // desiredAlignmentPosePublisher.set(desiredAlignmentPose);
        // desiredStatesPublisher.set(getModuleStates());
        actualStatesPublisher.set(getModuleStates());
        // swerveOdometry.update(getGyroYaw(), getModulePositions());
        // SmartDashboard.putBoolean("isAligned", isAligned());
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

    }

}
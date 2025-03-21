package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constField;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup, leftReef, rightReef;
    double elevatorMultiplier;
    Elevator elevator;
    double redAllianceMultiplier = 1;

    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier leftReef,
            BooleanSupplier rightReef) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.elevator = elevator;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftReef = leftReef;
        this.rightReef = rightReef;
    }

    @Override
    public void initialize() {
                redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("isRedAlliance", constField.isRedAlliance());
        elevatorMultiplier = MathUtil.clamp(1 - MathUtil.applyDeadband(elevator.getElevatorPosition().magnitude() / 60,
                Constants.constElevator.MULTIPLIER_DEADZONE), 0.35, 1);
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);
        // SmartDashboard.putNumber("Translation", translationVal);
        // SmartDashboard.putNumber("Strafe", strafeVal);
        // SmartDashboard.putNumber("Rotation", rotationVal);
        // SmartDashboard.putNumber("Elevator/Elevator Multiplier", elevatorMultiplier);

        // -- Velocities --
        LinearVelocity xVelocity = Units.MetersPerSecond.of(translationVal * elevatorMultiplier);
        LinearVelocity yVelocity = Units.MetersPerSecond.of(strafeVal * elevatorMultiplier);
        AngularVelocity rVelocity = Units.RadiansPerSecond
                .of(-rotationSup.getAsDouble() * Constants.Swerve.TURN_SPEED.in(Units.RadiansPerSecond)*elevatorMultiplier);

        // // -- Coral Station --
        // if (leftCoralStationFar.getAsBoolean()) {
        // Pose2d desiredCoralStation =
        // Constants.constField.POSES.LEFT_CORAL_STATION_FAR;
        // Distance coralStationDistance = Units.Meters
        // .of(s_Swerve.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

        // s_Swerve.autoAlign(coralStationDistance, desiredCoralStation, xVelocity,
        // yVelocity, rVelocity,
        // transMultiplier, isOpenLoop,
        // Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
        // DriverState.CORAL_STATION_AUTO_DRIVING,
        // DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
        // }

        // else if (leftCoralStationNear.getAsBoolean()) {
        // Pose2d desiredCoralStation =
        // Constants.constField.POSES.LEFT_CORAL_STATION_NEAR;
        // Distance coralStationDistance = Units.Meters
        // .of(s_Swerve.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

        // s_Swerve.autoAlign(coralStationDistance, desiredCoralStation, xVelocity,
        // yVelocity, rVelocity,
        // transMultiplier, isOpenLoop,
        // Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
        // DriverState.CORAL_STATION_AUTO_DRIVING,
        // DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
        // }

        // else if (rightCoralStationFar.getAsBoolean()) {
        // Pose2d desiredCoralStation =
        // Constants.constField.POSES.RIGHT_CORAL_STATION_FAR;
        // Distance coralStationDistance = Units.Meters
        // .of(s_Swerve.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

        // s_Swerve.autoAlign(coralStationDistance, desiredCoralStation, xVelocity,
        // yVelocity, rVelocity,
        // transMultiplier, isOpenLoop,
        // Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
        // DriverState.CORAL_STATION_AUTO_DRIVING,
        // DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
        // }

        // else if (rightCoralStationNear.getAsBoolean()) {
        // Pose2d desiredCoralStation =
        // Constants.constField.POSES.RIGHT_CORAL_STATION_NEAR;
        // Distance coralStationDistance = Units.Meters
        // .of(s_Swerve.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

        // s_Swerve.autoAlign(coralStationDistance, desiredCoralStation, xVelocity,
        // yVelocity, rVelocity,
        // transMultiplier, isOpenLoop,
        // Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
        // DriverState.CORAL_STATION_AUTO_DRIVING,
        // DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
        // }

        // -- Controlling --
        if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
            // Reef auto-align is requested
            Pose2d desiredReef = s_Swerve.getDesiredReef(leftReef.getAsBoolean());
            Distance reefDistance = Units.Meters
                    .of(s_Swerve.getPoseEstimator().getTranslation().getDistance(desiredReef.getTranslation()));

            // Begin reef auto align (rotationally, automatically driving, or w/ a driver
            // override)
            s_Swerve.autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, elevatorMultiplier,
                    true,
                    Constants.Swerve.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE);
            ;
            LimelightHelpers.setLEDMode_ForceOn(Constants.constVision.LIMELIGHT_NAMES[0]);
            LimelightHelpers.setLEDMode_ForceOn(Constants.constVision.LIMELIGHT_NAMES[1]);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(Constants.constVision.LIMELIGHT_NAMES[0]);
            LimelightHelpers.setLEDMode_ForceOff(Constants.constVision.LIMELIGHT_NAMES[1]);
            /* Drive */
            s_Swerve.drive(
                    new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond), yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)).times(Constants.Swerve.MAX_SPEED)
                            .times(elevatorMultiplier),
                    rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                    !robotCentricSup.getAsBoolean(),
                    true);
        }
    }
}
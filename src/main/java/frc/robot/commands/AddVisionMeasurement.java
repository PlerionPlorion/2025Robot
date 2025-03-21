// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constVision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AddVisionMeasurement extends Command {
  Swerve swerve;
  Limelight limelight;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;

    addRequirements(limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!RobotState.isDisabled()) {
      // Tells the limelight where we are on the field
      LimelightHelpers.SetRobotOrientation(constVision.LIMELIGHT_NAMES[0],
          swerve.getPoseEstimator().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(constVision.LIMELIGHT_NAMES[1],
          swerve.getPoseEstimator().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      AngularVelocity gyroRate = Units.DegreesPerSecond.of(swerve.getGyroRate());

      Optional<PoseEstimate> estimatedPose = limelight.determinePoseEstimate(gyroRate);
      if (estimatedPose.isPresent()) {
        swerve.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

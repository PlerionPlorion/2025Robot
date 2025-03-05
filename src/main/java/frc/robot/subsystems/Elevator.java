// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.reefPosition;
import frc.robot.Robot;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  private Distance lastDesiredPosition;

  Distance currentLeftPosition = Units.Inches.of(0);
  Distance currentRightPosition = Units.Inches.of(0);

  reefPosition currentReefPos;
  reefPosition desiredReefPos;

  @NotLogged
  PositionVoltage positionRequest;
  @NotLogged
  VoltageOut voltageRequest = new VoltageOut(0);

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;

  @NotLogged
  MotionMagicVoltage motionRequest;

  // File deployDirectory = Filesystem.getDeployDirectory();
  // private CvSource outputStream;
  // private Mat image;
  // String imagePath;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotorFollower = new TalonFX(Constants.constElevator.LEFT_MOTOR_FOLLOWER_ID, Constants.CAN_BUS_NAME);
    rightMotorLeader = new TalonFX(Constants.constElevator.RIGHT_MOTOR_LEADER_ID, Constants.CAN_BUS_NAME);

    lastDesiredPosition = Units.Inches.of(0);
    voltageRequest = new VoltageOut(0);
    motionRequest = new MotionMagicVoltage(0);

    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    currentReefPos = reefPosition.NONE;

    // imagePath = Filesystem.getDeployDirectory().getAbsolutePath() +
    // "/ReefDisplay/image.png";
    // image = Imgcodecs.imread(imagePath);
    // outputStream = CameraServer.putVideo("Requested Reef Position",
    // image.width(), image.height());
    // outputStream.setFPS(2);
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());
  }

  public boolean isAtSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    } else {
      return (getElevatorPosition() // Checking if the move request set the elevator in the correct position (after
                                    // movement).
          .compareTo(getLastDesiredPosition().minus(Constants.constElevator.DEADZONE_DISTANCE)) > 0) &&
          getElevatorPosition().compareTo(getLastDesiredPosition().plus(Constants.constElevator.DEADZONE_DISTANCE)) < 0;
    }
  }

  public AngularVelocity getRotorVelocity() {
    return rightMotorLeader.getRotorVelocity().getValue();
  }

  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setPosition(Distance height) {
    rightMotorLeader.setControl(motionRequest.withPosition(height.in(Units.Inches)));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
    lastDesiredPosition = height;
  }

  public void setCoastMode(Boolean coastMode) {
    if (coastMode) {
      rightMotorLeader.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
      leftMotorFollower.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
    } else {
      rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
      leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    }
  }

  // public void setReefDisplay() {
  // switch (currentReefPos) {
  // case L1:
  // imagePath = Filesystem.getDeployDirectory().getAbsolutePath() +
  // "/ReefDisplay/ReefL1.png";
  // break;
  // case L2:
  // imagePath = Filesystem.getDeployDirectory().getAbsolutePath() +
  // "/ReefDisplay/ReefL2.png";
  // break;
  // case L3:
  // imagePath = Filesystem.getDeployDirectory().getAbsolutePath() +
  // "/ReefDisplay/ReefL3.png";
  // break;
  // case L4:
  // imagePath = Filesystem.getDeployDirectory().getAbsolutePath() +
  // "/ReefDisplay/ReefL4.png";
  // break;
  // }
  // image = Imgcodecs.imread(imagePath);
  // outputStream.putFrame(image);
  // }

  public void setReefCycle(boolean invert) {
    switch (currentReefPos) {
      case NONE:
        desiredReefPos = invert ? reefPosition.NONE : reefPosition.L1;
        break;
      case L1:
        desiredReefPos = invert ? reefPosition.NONE : reefPosition.L2;
        break;
      case L2:
        desiredReefPos = invert ? reefPosition.L1 : reefPosition.L3;
        break;
      case L3:
        desiredReefPos = invert ? reefPosition.L2 : reefPosition.L4;
        break;
      case L4:
        desiredReefPos = invert ? reefPosition.L3 : reefPosition.L4;
        break;
    }
    setReefPosition(desiredReefPos);
    // setReefDisplay();
  }

  public void setReefPosition(reefPosition desiredReefPosition) {
    setPosition(Units.Inches.of(
        desiredReefPos == reefPosition.NONE ? 0.0
            : desiredReefPos == reefPosition.L1 ? 22.0
                : desiredReefPos == reefPosition.L2 ? 29.5
                    : desiredReefPos == reefPosition.L3 ? 43.5
                        : desiredReefPos == reefPosition.L4 ? 67 : 0.0));
                        currentReefPos = desiredReefPos;
    // setReefDisplay();
  }

  public reefPosition getReefPosition() {
    return currentReefPos;
  }

  public void setNeutral() {
    rightMotorLeader.setControl(new NeutralOut());
    leftMotorFollower.setControl(new NeutralOut());
  }

  public void setVoltage(Voltage voltage) {
    rightMotorLeader.setControl(voltageRequest.withOutput(voltage));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), false));
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public void resetSensorPosition(Distance setpoint) {
    rightMotorLeader.setPosition(setpoint.in(Inches));
    leftMotorFollower.setPosition(setpoint.in(Inches));
  }

  @Override
  public void periodic() {

    if(currentReefPos != null) {
    SmartDashboard.putString("Reef Position", currentReefPos.toString());
    }

    // This method will be called once per scheduler run
    currentLeftPosition = Units.Inches.of(leftMotorFollower.getPosition().getValueAsDouble());
    currentRightPosition = Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Left/CLO", leftMotorFollower.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", leftMotorFollower.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", leftMotorFollower.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", leftMotorFollower.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Right/CLO", rightMotorLeader.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", rightMotorLeader.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", rightMotorLeader.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", rightMotorLeader.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Elevator/atSetpoint", isAtSetpoint());
    SmartDashboard.putNumber("Elevator/Position", getElevatorPosition().magnitude());
    SmartDashboard.putNumber("Elevator/LastPosition", getLastDesiredPosition().magnitude());
  }
}

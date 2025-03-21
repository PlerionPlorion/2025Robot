// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.TalonFXS;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.constAlgaeIntake;

// @Logged
// public class AlgaeIntake extends SubsystemBase {
//   TalonFXS intakeRollerMotor;
//   TalonFX intakePivotMotor;

//   private Angle lastDesiredAngle = Degrees.zero();

//   PositionVoltage positionRequest = new PositionVoltage(0);
//   VoltageOut voltageRequest = new VoltageOut(0);
//   MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

//   public boolean attemptingZeroing = false;
//   public boolean hasZeroed = false;
//   public boolean hasAlgaeOverride = false;

//   /** Creates a new AlgaeIntake. */
//   public AlgaeIntake() {
//     intakeRollerMotor = new TalonFXS(constAlgaeIntake.INTAKE_ROLLER_MOTOR_CAN, Constants.CAN_BUS_NAME);
//     intakePivotMotor = new TalonFX(constAlgaeIntake.INTAKE_PIVOT_MOTOR_CAN, Constants.CAN_BUS_NAME);

//     intakeRollerMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_ROLLER_CONFIG);
//     intakePivotMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_PIVOT_CONFIG);
//   }

//   public void setAlgaeIntakeMotor(double speed) {
//     intakeRollerMotor.set(speed);
//   }

//   public void setAlgaePivotAngle(Angle setpoint) {
//     intakePivotMotor.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)));
//     lastDesiredAngle = setpoint;
//   }

//   public AngularVelocity getRotorVelocity() {
//     return intakePivotMotor.getRotorVelocity().getValue();
//   }

//   public boolean isRotorVelocityZero() {
//     return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
//   }

//   public void setVoltage(Voltage voltage) {
//     intakePivotMotor.setControl(voltageRequest.withOutput(voltage));
//   }

//   public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
//     constAlgaeIntake.ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
//     constAlgaeIntake.ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

//     intakePivotMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_PIVOT_CONFIG);
//   }

//   public Angle getPivotAngle() {
//     return intakePivotMotor.getPosition().getValue();
//   }

//   public Angle getLastDesiredPivotAngle() {
//     return lastDesiredAngle;
//   }

//   public void resetSensorPosition(Angle zeroedPos) {
//     intakePivotMotor.setPosition(zeroedPos);
//   }

//   public boolean hasAlgae() {
//     Current intakeCurrent = intakeRollerMotor.getStatorCurrent().getValue();

//     AngularVelocity intakeVelocity = intakeRollerMotor.getVelocity().getValue();
//     double intakeAcceleration = intakeRollerMotor.getAcceleration().getValueAsDouble();

//     Current intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_CURRENT;
//     AngularVelocity intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;

//     if (hasAlgaeOverride) {
//       return hasAlgaeOverride;
//     }

//     if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
//         && (intakeVelocity.lte(intakeHasGamePieceVelocity))
//         && (intakeAcceleration < 0)) {
//       return true;
//     } else {
//       return false;
//     }
//   }

//   public void setHasAlgaeOverride(boolean passedHasGamePiece) {
//     hasAlgaeOverride = passedHasGamePiece;
//   }

//   public void algaeToggle() {
//     this.hasAlgaeOverride = !hasAlgaeOverride;
//   }

//   public double getAlgaeIntakeVoltage() {
//     return intakeRollerMotor.getMotorVoltage().getValueAsDouble();
//   }

//   public void setAlgaeIntakeVoltage(double voltage) {
//     intakeRollerMotor.setVoltage(voltage);
//   }

//   public boolean isAtSetPoint() {
//     return (getPivotAngle()
//         .compareTo(getLastDesiredPivotAngle().minus(constAlgaeIntake.DEADZONE_DISTANCE)) > 0) &&
//         getPivotAngle().compareTo(getLastDesiredPivotAngle().plus(constAlgaeIntake.DEADZONE_DISTANCE)) < 0;
//   }

//   @Override
//   public void periodic() {
//     // SmartDashboard.putNumber("AlgaeAngle", getPivotAngle().in(Units.Degrees));
//   }
// }

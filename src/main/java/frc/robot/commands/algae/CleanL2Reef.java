// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.algae;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.constAlgaeIntake;
// import frc.robot.Constants.reefPosition;
// import frc.robot.subsystems.AlgaeIntake;
// import frc.robot.subsystems.Elevator;;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CleanL2Reef extends Command {
//   Elevator elevator;
//   AlgaeIntake algaeIntake;

//   public CleanL2Reef(Elevator elevator, AlgaeIntake algaeIntake) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.algaeIntake = algaeIntake;
//     this.elevator = elevator;
//     addRequirements(algaeIntake, elevator);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     elevator.setPosition(Constants.constElevator.ALGAE_L2_CLEANING);
//     algaeIntake.setAlgaeIntakeMotor(Constants.constAlgaeIntake.ALGAE_INTAKE_SPEED);
//     algaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     if (!algaeIntake.hasAlgae()) {
//       algaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
//       elevator.setReefPosition(reefPosition.NONE);
//     }
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

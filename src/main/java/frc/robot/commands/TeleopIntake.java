
package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constIntake;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
  /** Creates a new TeleopOuttake. */
  Intake intake;

  public TeleopIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setVoltage(constIntake.INTAKE_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
    intake.setPosition(Units.Inches.of(6));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.Elevator;

public class ManualZeroElevator extends Command {
  Elevator globalElevator;

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);

  public static boolean hasSetCoastMode = false;


  AngularVelocity lastRotorVelocity = Units.RotationsPerSecond.of(0);

  public ManualZeroElevator(Elevator subElevator) {
    this.globalElevator = subElevator;

    addRequirements(subElevator);
  }

  @Override
  public void initialize() {
    globalElevator.setSoftwareLimits(false, true);
    globalElevator.hasZeroed = false;
  }

  @Override
  public void execute() {

    if (!hasSetCoastMode) {
      globalElevator.setCoastMode(true);
      hasSetCoastMode = true;
    }

    // Check if we have raised the elevator above a certain speed
    if (globalElevator.getRotorVelocity().gte(constElevator.MANUAL_ZEROING_START_VELOCITY)
        || globalElevator.attemptingZeroing) {
      // Enter zeroing mode!
      if (!globalElevator.attemptingZeroing) {
        globalElevator.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Elevator Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constElevator.ZEROING_TIMEOUT)) {
        globalElevator.attemptingZeroing = false;
        System.out.println("Elevator Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = globalElevator.getRotorVelocity().minus(lastRotorVelocity)
            .lte(constElevator.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = globalElevator.getRotorVelocity();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    globalElevator.setSoftwareLimits(true, true);

    if (!interrupted) {
      globalElevator.hasZeroed = true;
      globalElevator.resetSensorPosition(constElevator.ZEROED_POS);
      globalElevator.setCoastMode(false);
      System.out.println("Elevator Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Elevator was never zeroed :((( blame eli");
    }
  }

  @Override
  public boolean isFinished() {
    return zeroingSuccess && globalElevator.isRotorVelocityZero();
  }
}

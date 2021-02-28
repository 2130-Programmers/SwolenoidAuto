// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ModifierSub extends SubsystemBase {
  /** Creates a new ModifierSub. */

  public boolean stat;
  public double bestMod;

  public double trigger;

  public ModifierSub() {
    stat = false;
    bestMod = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    trigger = RobotContainer.driverJoystick.getRawAxis(Constants.driverRightAxisTrigger);
  }

  public void finalMod(double mod, double triggerMod) {
    if (trigger * triggerMod >= mod) {
      bestMod = trigger * triggerMod;
    } else {
      bestMod = mod;
    }
  }

}
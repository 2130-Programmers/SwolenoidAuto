/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  /**
   * Creates a new LauncherSubsystem.
   */
  public CANSparkMax launcherMotorMaster;
  public CANSparkMax launcherMoterSlave;

  public double finalSpeed;
  public boolean active;

  public LauncherSubsystem() {

    launcherMotorMaster = new CANSparkMax(2, MotorType.kBrushless);
    launcherMoterSlave = new CANSparkMax(1, MotorType.kBrushless);

    finalSpeed = 0;
    active = false;

    setMaster();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void setMaster(){
    launcherMoterSlave.follow(launcherMotorMaster);
  }

  public void windUp(){
    finalSpeed +=.01;
    launcherMotorMaster.set(finalSpeed);
  }

  public void windDown(){
    finalSpeed = .0;
    launcherMotorMaster.set(0);
  }

  public void speedReset(){
    finalSpeed = 0;
  }
}

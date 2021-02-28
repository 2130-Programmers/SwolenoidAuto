// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutonomousStep;
import frc.robot.subsystems.DriveTrain;

public class Autonomous extends CommandBase {
  /* Creates a new Drive. */

  private DriveTrain driveTrain;

  private double autoStartTime;
  private double currentTime;



  //Creating steps
  AutonomousStep driveForward1 = new AutonomousStep(10, 15);
  AutonomousStep stop = new AutonomousStep(16, 20);

  public Autonomous(DriveTrain drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesub);
    driveTrain = drivesub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //RobotContainer.driveTrain.findAllZeros();
    RobotContainer.gyro.calibrate();


    autoStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    RobotContainer.driveTrain.setAutonomousState(true);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveForward1.run(.75, 0, 0);
    stop.stopAll();

    SmartDashboard.putNumber("CurrentTime", currentTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.setAutonomousState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
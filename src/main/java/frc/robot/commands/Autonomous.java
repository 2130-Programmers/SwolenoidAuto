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

  // CREATE STEPS FOR AUTONOMOUS HERE ~~ SYNTAX: AutonomousStep (name of step,
  // repeating things like drive forward just have a number appended to the end.)
  // = new AutonomousStep(startTime, endTime)
  //
  //If no args in constructor, 
  AutonomousStep startAutoTimer = new AutonomousStep();
  AutonomousStep driveForward1 = new AutonomousStep(10, 20);
  AutonomousStep stop = new AutonomousStep(20, 21);
  AutonomousStep resetAutoTimer = new AutonomousStep();

  public Autonomous(DriveTrain drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesub);
    driveTrain = drivesub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
    //~~~~~~~~Set Steps Here~~~~~~~~\\
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

    //Stepname.method
    //.run for strafe or rotate
    //.stopAll to stop all motors
    //.resetGyro to zero the gyro

    driveForward1.run(1, .1, 0);
    stop.stopAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.setAutonomousState(false);
    resetAutoTimer.resetAutonomousTime();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
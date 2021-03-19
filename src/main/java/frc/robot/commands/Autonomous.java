// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutonomousStep;
import frc.robot.subsystems.DriveTrain;

public class Autonomous extends CommandBase {
  /* Creates a new Drive. */

  private DriveTrain driveTrain;

  // DEPRECATED! DO NOT USE UNLESS REVERTED BACK FROM STATIC CLASSES
  //
  // CREATE STEPS FOR AUTONOMOUS HERE ~~ SYNTAX: AutonomousStep (name of step,
  // repeating things like drive forward just have a number appended to the end.)
  // = new AutonomousStep(startTime, endTime)
  //
  // If no args in constructor, you can put a step in AutonomousInit

  AutonomousStep drive1 = new AutonomousStep();
  AutonomousStep drive2 = new AutonomousStep();
  AutonomousStep drive3 = new AutonomousStep();
  AutonomousStep drive4 = new AutonomousStep();
  AutonomousStep drive5 = new AutonomousStep();
  AutonomousStep stop = new AutonomousStep();

  // AutonomousStep circle = new AutonomousStep(0, 10);
    AutonomousStep resetAutoTimer = new AutonomousStep();

  public Autonomous(DriveTrain drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesub);
    driveTrain = drivesub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveTrain.setAutonomousState(true);
    resetAutoTimer.resetAutoClock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
    // ~~~~~~~~Set Steps Here~~~~~~~~\\
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

    // Stepname.method
    // .run for strafe or rotate
    // .stopAll to stop all motors
    // .resetGyro to zero the gyro

   drive1.run(0, .25, 1000);

  // circle.circle(true);

    RobotContainer.driveTrain.autonomousTimer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.setAutonomousState(false);
    resetAutoTimer.resetAutoClock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
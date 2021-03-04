// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AutonomousStep {
    double startTime; // Time to start running step.
    double endTime; // Time to stop running step

    public AutonomousStep(double startTime, double endTime) {
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public AutonomousStep() {

    }

    public void resetAutonomousTime() {
        RobotContainer.driveTrain.autonomousStartTime = Timer.getFPGATimestamp();
        RobotContainer.driveTrain.autonomousRunningTime = 0;
    }

    public void run(double x, double y, double rotation) {

        double rotationPower = 0;

        if (rotation != 0) {
            double remainingRotation = rotation - RobotContainer.gyro.getAngle();

            if (remainingRotation > 45) {
                rotationPower = -.3;
            } else if (remainingRotation < 45 && remainingRotation > 3) {
                rotation = -.05;
            }

            if (RobotContainer.gyro.getAngle() < -45) {
                rotationPower = .3;
            } else if (remainingRotation > -45 && remainingRotation < -3) {
                rotationPower = .05;
            }

            if (Math.abs(RobotContainer.gyro.getAngle()) < 3) {
                rotationPower = 0;
            }

        } else if (rotation == 0) {

            if (RobotContainer.gyro.getAngle() > 3) {
                rotationPower = -.05;
            } else if (RobotContainer.gyro.getAngle() < -3) {
                rotationPower = .05;
            }
        }

        RobotContainer.driveTrain.autonomousRunningTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
                - RobotContainer.driveTrain.autonomousStartTime;

        if (RobotContainer.driveTrain.autonomousRunningTime > this.startTime && RobotContainer.driveTrain.autonomousRunningTime < this.endTime) {
            RobotContainer.driveTrain.moveSwerveAxis(y, x*-1, rotationPower);
        }

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Rotation Power", rotationPower);
        SmartDashboard.putNumber("Running Time", RobotContainer.driveTrain.autonomousRunningTime);
    }

    public void circle(boolean leftOrRight) {
        double gradientX = 0;
        double gradientY = -1;

        gradientY += .1;
        double yDir = gradientY / Math.abs(gradientY);

        double finalY = yDir * (gradientY * gradientY) - .5;

        RobotContainer.driveTrain.moveSwerveAxis(0, finalY, 0);
    }

    public void resetGyro() {
        RobotContainer.gyro.reset();
    }

    public void stopAll() {
        RobotContainer.driveTrain.stopAllMotors();
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AutonomousStep {

    double rotationPower = 0;
    boolean stepDone;

    public AutonomousStep() {
    }

     /**
      * @param direction The strafe direction for the robot
                takes 45 degree intervals starting at 0
      * @param power Power modifier - takes a value of 0 - 1
      * @param distance The distance in stops that you want the robot to travel
      */
    public void run(int direction, double power, double distance) {
        double y = 0;
        double x = 0;

        if (RobotContainer.gyro.getAngle() > 3) {
            rotationPower = -.05;
        } else if (RobotContainer.gyro.getAngle() < -3) {
            rotationPower = .05;
        }

        double distanceRemaining = distance - Math.abs(((RobotContainer.driveTrain.motorFR.encoderCount()
                + RobotContainer.driveTrain.motorFL.encoderCount() + RobotContainer.driveTrain.motorRR.encoderCount()
                + RobotContainer.driveTrain.motorRL.encoderCount()) / 4));

        if (distanceRemaining > 20) {
            switch (direction) {
                case 0:
                    y = 1;
                    x = 0;
                    break;
                case 45:
                    y = 1;
                    x = 1;
                    break;
                case 90:
                    y = 0;
                    x = 1;
                    break;
                case 135:
                    y = -1;
                    x = 1;
                    break;
                case 180:
                    y = -1;
                    x = 0;
                    break;
                case 225:
                    y = -1;
                    x = -1;
                    break;
                case 270:
                    y = 0;
                    x = -1;
                    break;
                case 315:
                    y = 1;
                    x = -1;
                    break;
                default:
                    y = 0;
                    x = 0;
                    break;
            }
        } else if (distanceRemaining < 20 && distanceRemaining > 2) {
            power = .1;
        } else if (distanceRemaining < 2) {
            power = 0;
        }

        stepDone = true;

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);

        RobotContainer.driveTrain.moveSwerveAxis(y * power, x * (-1 * power), rotationPower);
    }

    public void rotate(double rotation) {

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

        } else {
            System.out.print("Rotation Needs a Valid Value");
        }
    }

    public void resetAutoClock() {
        RobotContainer.driveTrain.autonomousStartTime = Timer.getFPGATimestamp();
        RobotContainer.driveTrain.autonomousRunningTime = 0;
    }

    public void circle(boolean leftOrRight) {

        double circleX = 0;
        double angle = 0;
        boolean circleDone = false;

        /**
         * @param leftOrRight - (boolean) If you are going right put a true and if the
         *                    direction is left put a false
         */

        if (circleX <= 1.5) {
            circleX += .008;
            circleDone = false;
            angle = ((.124 * Math.sin(14.2 * circleX + 1.6) + 1.8 * circleX - .2) / 1.6);
        } else {
            circleX = 0;
            circleDone = true;
        }

        if (leftOrRight == false) {
            angle = Math.abs(angle - 2);
        }

        RobotContainer.driveTrain.motorFL.drive(.35 * -1, angle, 1);
        RobotContainer.driveTrain.motorFR.drive(.35, angle, 1);
        RobotContainer.driveTrain.motorRR.drive(.35, angle, 1);
        RobotContainer.driveTrain.motorRL.drive(.35 * -1, angle, 1);
    }

    public void resetGyro() {
        RobotContainer.gyro.reset();
    }

    public void stopAll() {
        RobotContainer.driveTrain.stopAllMotors();
    }

    public boolean stepState() {
        return stepDone;
    }
}
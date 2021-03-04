/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlphaMotors extends SubsystemBase {
    /**
     * Creates a new AlphaMotor.
     */

    private TalonFX driveMotor;
    private TalonSRX rotationMotor; // The directional motor for the use of the
                                    // drive motor.

    private Encoder rotationEncoder; // The encoder for the use of the
                                     // turreting motor.
    private DigitalInput rotationProx; // The proximity sebsor for the use of
                                       // the turreting motor.

    public long directionTarget;

    public double encoderRemainingValue;

    public boolean inMethod;

    public double deleteMe;

    
    public double pointSet = 0;
    /**
     * Sets the min and max output of the turreting motor
     * 
     * @param forward - (double) The forward peak output for the motor.
     * @param reverse - (double) The reverse peak output for the motor.
     * @return (void)
     */

    private void setMinMaxOutput(double forward, double reverse) {

        rotationMotor.configPeakOutputForward(forward);
        rotationMotor.configPeakOutputReverse(reverse);

    }

    /**
     * Takes the inputs and assigns the given source IDs to the corresponding
     * device.
     * 
     * @param motorDeviceNumber - (int) CAN Device ID of the TalonSRX .
     * @param encSourceA        - (int) The a channel digital input for the encoder.
     * @param encSourceB        - (int) The b channel digital input for the encoder.
     * @param proxChannel       - (int) The DIO channel digital input for the prox.
     * @return (void)
     */

    public AlphaMotors(int rotate, int speed, int encSourceA, int encSourceB, int proxChannel) {
        /**
         * Takes the inputs and assigns the given source IDs to the corresponding
         * device.
         */

        rotationMotor = new TalonSRX(rotate); //
        driveMotor = new TalonFX(speed);

        rotationEncoder = new Encoder(encSourceA, encSourceB);
        rotationProx = new DigitalInput(proxChannel);

        this.setMinMaxOutput(Constants.TURRET_SPEED_MAX_OUTPUT, Constants.TURRET_SPEED_MIN_OUTPUT);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Grabs the current encoder count (in ticks) and reseting after crossing 1
     * rotations worth of ticks
     * 
     * @return (int) The rotation motor's encoder count
     */

    public double currentEncoderCount() {

        if (rotationEncoder.get() >= 0) {
            return rotationEncoder.get();
        } else {
            return rotationEncoder.get() + 420;
        }

    }

    /**
     * Applies a desired power level to the rotation motor
     * 
     * @param speed - (double) the desired power in percentage [-1, 1]
     * 
     * @return (void)
     */

    private void moveMotor(double speed) {
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Applies a power of 0 to the rotation motor
     * 
     * @return (void)
     */

    private void stopMotors() {
        rotationMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Takes a joystick input and turns the rotation motor to the equivilant of that
     * position in ticks
     * 
     * @param targetX - (double) The joystick's current x-value
     * @param targetY - (double) The joystick's current y-value
     * 
     * @return (void)
     */

    public void pointToTarget(double target) {

        double currentPosition = currentEncoderCount();
        double desiredTarget = target; // The target position in ticks
        encoderRemainingValue = desiredTarget - currentPosition; // The distance between the current position and the
                                                                 // desired target
        double directionalMultiplier = 0; // A placeholder to determine the direction of the motor

        inMethod = false;

        if ((encoderRemainingValue != 0) || (encoderRemainingValue - 420 != 0) || (encoderRemainingValue + 420 != 0)) {
            // As long as we actually have a different target to go to, we continue to
            // actually go to the desired target

            inMethod = true;

            if (encoderRemainingValue > 210) {
                directionalMultiplier = Math
                        .round((encoderRemainingValue - 420) / Math.abs(encoderRemainingValue - 420));
            } else if (encoderRemainingValue < -210) {
                directionalMultiplier = Math
                        .round((encoderRemainingValue + 420) / Math.abs(encoderRemainingValue + 420));
            } else if (encoderRemainingValue < 210 && encoderRemainingValue > -210 && encoderRemainingValue != 0) {
                directionalMultiplier = Math.round((encoderRemainingValue) / Math.abs(encoderRemainingValue));
            //} else if (desiredTargetTicks(targetX, targetY) == 0){
            //    directionalMultiplier = 1;
            }else{
                //Change this to 0 later for testing-cory
                directionalMultiplier = 1;
            }

            if (Math.abs(encoderRemainingValue) > Constants.LARGE_SWERVE_ROTATION_ERROR) {
                moveMotor(Constants.FAST_SWERVE_ROTATION_SPEED * -directionalMultiplier);
            } else if (Math.abs(encoderRemainingValue) > Constants.SMALL_SWERVE_ROTATION_ERROR) {
                moveMotor(Constants.SLOW_SWERVE_ROTATION_SPEED * -directionalMultiplier);
            } else {
                stopMotors();
            }

        }
    }

    public void zeroEncoderBasedOnProx() {
        if (proxValue()) {
            zeroEncoder();
        }
    }

    public void zeroEncoder() {
        rotationEncoder.reset();
    }

    public boolean proxValue() {
        return !rotationProx.get();
    }

    public int encoderValue() {
        return rotationEncoder.get();
    }

    public void findZero() {
        int i = 0;
        while (!proxValue()) {
            double speed = Constants.FAST_SWERVE_ROTATION_SPEED;
            if (encoderValue() < 0 && i == 0) {
                speed = -speed;
                i++;
            }
            moveMotor(speed);
        }
        zeroEncoder();
        swerveDatBoi(0);
        i = 0;
    }

    public void swerveDatBoi(long desiredTarget) {
        if (encoderRemaining(desiredTarget, true) < Constants.SLOW_SWERVE_ROTATION_SPEED) {
            stopMotors();
        } else if (encoderRemaining(desiredTarget, true) < Constants.FAST_SWERVE_ROTATION_SPEED) {
            moveMotor(Constants.SLOW_SWERVE_ROTATION_SPEED
                    * (encoderRemaining(desiredTarget, false) / encoderRemaining(desiredTarget, true)));
        } else {
            moveMotor(Constants.FAST_SWERVE_ROTATION_SPEED
                    * (encoderRemaining(desiredTarget, false) / encoderRemaining(desiredTarget, true)));
        }
    }

    private long encoderRemaining(long targetValue, boolean abs) {

        long encRem;

        if (abs) {
            encRem = Math.abs(targetValue - encoderValue());
        } else {
            encRem = targetValue - encoderValue();
        }

        return encRem;
    }


    // The new swerve computations for angle and final speed
    public void drive(double speed, double angle, double mod) {

        double revamp = speed;
        
        //making it so speed can only return 1 or less* the mod because x/x = 1 and 
        //the abs makes the direction maintain true abs(x)/-x= -1
        if(1 < Math.abs(speed)){
            revamp = (speed/Math.abs(speed)*mod);
        }else{
            revamp = (speed)*mod;
        }

        driveMotor.set(ControlMode.PercentOutput, revamp);

        //converting the angle from the new swerve function into terms communicable with the rest of the code
        if(angle < 0){
            //This sets the angle into positive by flipping the value on the origin then multiplying to get it to terms of 420
            pointSet = (Math.abs(1 + angle) + 1) * 210;
        }else{
            pointSet = angle * 210;
        }

        //shoves the angle into the rest of the code
        pointToTarget(pointSet);    

        deleteMe = pointSet;
    }

    public void brodieAuto(double speed, double angle) {
        driveMotor.set(ControlMode.PercentOutput, speed);

        double pointS = angle*1.6777777777777777777777777777777777777777777777777;

        pointToTarget(pointS);
    }
}
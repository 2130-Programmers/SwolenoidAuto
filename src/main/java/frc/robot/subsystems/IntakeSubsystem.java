/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * Creates a new IntakeSubsystem.
     */

    private TalonSRX intakeMotor;

    private DoubleSolenoid intakeSol;
    private Solenoid feedSol;

    public IntakeSubsystem() {

        intakeMotor = new TalonSRX(10);

        intakeSol = new DoubleSolenoid(3, 4);
        feedSol = new Solenoid(1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runIntake() {
        intakeMotor.set(ControlMode.PercentOutput,
                RobotContainer.getOperatorAxis(3) - RobotContainer.getOperatorAxis(2));
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void flopIntakeOut() {
        intakeSol.set(Value.kForward);
    }

    public void flopIntakeIn() {
        intakeSol.set(Value.kReverse);
    }

    public void leaveIntakeNeutral() {
        intakeSol.set(Value.kOff);
    }

    public void releaseBalls() {
        feedSol.set(true);
    }

    public void tightenBalls() {
        feedSol.set(false);
    }

    public void masterOfBalls(){
        if(RobotContainer.operatorJoystick.getRawAxis(3)>.9){
            releaseBalls();
        }else{
            tightenBalls();
        }
    }

}

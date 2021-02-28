/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DriveTrain driveTrain = new DriveTrain();
  public final static ModifierSub modifierSub = new ModifierSub();
  public final static LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  public final static SensorsSubsystem sensorsSubsystem = new SensorsSubsystem();
  public final static AimingSubsystem aimingSubsystem = new AimingSubsystem();
  public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final PlsWork plsWork = new PlsWork(driveTrain);
  private final ModChanger modChanger = new ModChanger(modifierSub);
  private final HeresTheWindUp heresTheWindUp = new HeresTheWindUp(launcherSubsystem);
  private final AimingCommand aimingCommand = new AimingCommand(aimingSubsystem);
  private final AimingCommandDown aimingCommandDown = new AimingCommandDown(aimingSubsystem);
  private final DisengageStopBallSolenoid disengageStopBallSolenoid = new DisengageStopBallSolenoid(intakeSubsystem);
  private final AimSetpoionts aimSetpoionts = new AimSetpoionts(aimingSubsystem);

  //Autonomous Command
  private final Autonomous autonomous = new Autonomous(driveTrain);


  public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  /**
   * The Driver Joystick declaration and the button definitions associated with
   * it.
   */

  public static final Joystick driverJoystick = new Joystick(0);

  private final static JoystickButton aimWithALime = new JoystickButton(driverJoystick, Constants.driverButtonLB);
  /**
   * The Operator Joystick declaration and the button definitions associated with
   * it.
   */

  public static final Joystick operatorJoystick = new Joystick(1);

  private final static JoystickButton changeHandlerPositionButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonRightJoyClick);
  private final static JoystickButton windLauncherUpButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonX);
  private final static JoystickButton windLauncherDownButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonY);
  private final static JoystickButton lowerLauncherButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonLB);
  private final static JoystickButton raiseLauncherButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonRB);
  private final static JoystickButton runWinchButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonBack);
  private final static JoystickButton disengageStopBallSoneloidButton = new JoystickButton(operatorJoystick,
      Constants.operatorButtonLeftJoyClick);

  // cardinal directions on the dpad and they work in angles starting with 0 on
  // top
  private final static POVButton zoneThree = new POVButton(operatorJoystick, 90);
  private final static POVButton zoneFour = new POVButton(operatorJoystick, 270);
  private final static POVButton bottomZone = new POVButton(operatorJoystick, 180);
  private final static POVButton up = new POVButton(operatorJoystick, 0);

  /*
  Dpad is a class I created that serves as a button that activats when any of the documented direction of the dpad is pressed
  It is loacted below constants
  */
  private final static Dpad dpad = new Dpad(up, bottomZone, zoneFour, zoneThree);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(plsWork);
//    modifierSub.setDefaultCommand(modChanger);
    intakeSubsystem.setDefaultCommand(disengageStopBallSolenoid);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    windLauncherUpButton.whenPressed(heresTheWindUp, true);

    raiseLauncherButton.whenPressed(aimingCommand, true);
    lowerLauncherButton.whenPressed(aimingCommandDown, true);

    disengageStopBallSoneloidButton.whenPressed(disengageStopBallSolenoid, true);

    dpad.whenPressed(aimSetpoionts, true);

    zoneThree.whenPressed(aimSetpoionts, true);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomous;
  }
  public static double getDriverAxis(int axis) {
    if (axis == 1 || axis == 5) {
      return -driverJoystick.getRawAxis(axis);
    } else {
      return driverJoystick.getRawAxis(axis);
    }
  }

  public static double getOperatorAxis(int axis) {
     if (axis == 1 || axis == 5) {
         return -operatorJoystick.getRawAxis(axis);
        } else {
         return operatorJoystick.getRawAxis(axis);
        }
    }

    public static boolean limeValue(){
      return aimWithALime.get();
    }

    //operator button values

    public static boolean handlerPositionValue() {
        return changeHandlerPositionButton.get();
      }
    
      public static boolean climbButtonValue() {
        return runWinchButton.get();
      }
    
      public static boolean launcherButVal(){
        return windLauncherUpButton.get();
      }
    
      public static boolean stopWindin(){
        return windLauncherDownButton.get();
      }
    
      public static boolean lowerButVal(){
        return lowerLauncherButton.get();
      }
      public static boolean raiseButVal(){
        return raiseLauncherButton.get();
      }
    
      public static boolean disengageStopBallSoneloidButtonValue() {
        return disengageStopBallSoneloidButton.get();
      }

      //returns which dpad side is getting pressed.
      public static int dpadValue(){
        if(up.get()){
          return 3;
        }else if(bottomZone.get()){
          return 1;
        }else if(zoneThree.get()){
          return 2;
        }else if(zoneFour.get()){
          return 4;
        }else{
          return 0;
        }
      }

}


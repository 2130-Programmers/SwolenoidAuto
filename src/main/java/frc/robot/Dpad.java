// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;


//I made the Dpad a button type here as shown in subsystems they would be a subsystem -cory
public class Dpad extends Button {
  /** Creates a new Dpad. */
  public static boolean pup;
  public static boolean pdown;
  public static boolean pleft;
  public static boolean pright;

  public Dpad(POVButton up, POVButton down, POVButton left, POVButton right){
    pup = up.get();
    pdown = down.get();
    pright = right.get();
    pleft = left.get();
  }
    
    public static boolean isDpadActive(){
      if(pup || pdown || pleft || pright ){
        return true;
      }else{
        return false;
      }
    }
}

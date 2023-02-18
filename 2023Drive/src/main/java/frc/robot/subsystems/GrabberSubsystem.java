// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {}

     private Solenoid Grabswitch = new Solenoid(PneumaticsModuleType.REVPH, 0);

     public void Grab(int setting){
      Grabswitch.set(true);
       
     }
     public void Ungrab(int setting){
      Grabswitch.set(false);
       
     }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

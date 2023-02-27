
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakSubsystem extends SubsystemBase {
  /** Creates a new BreakSubsystem. */
  private Solenoid Breakswitch = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  private boolean breakState = false;

  public BreakSubsystem() {
    Breakswitch.set(false);
  }
  
  public void ToggleBreak() {
    breakState = breakState == false ? true : false;
    if (breakState == false) {
      Unbreak();
    } else if (breakState == true) {
      Break();
    }
  }

  public void Break() {
    Breakswitch.set(true);

  }

  public void Unbreak() {
    Breakswitch.set(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

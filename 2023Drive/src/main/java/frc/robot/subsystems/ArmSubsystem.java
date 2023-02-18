// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.Object;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.AutoCloseable;
import frc.robot.Constants;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private TimeOfFlight armDistance;
  private CANSparkMax armExtend;
  private boolean extendOverride;
  public ArmSubsystem() {
    armDistance = new TimeOfFlight(Constants.TIMEOFFLIGHT_ID);
    armExtend = new CANSparkMax(6, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getArmDistance() {
    double distance = armDistance.getRange();
    System.out.println(distance);
    if (distance > 1699){
      return 2000;
    }
    return distance;
   }
  public void extend(double extendSpeed){
    boolean lessThanMax = this.getArmDistance() < Constants.MAX_DISTANCE;
    boolean moreThanBegin = this.getArmDistance() > Constants.ARM_BEGIN;
    boolean retracting = extendSpeed < 0;
    if (this.extendOverride){
      armExtend.set(extendSpeed);
    }
    else if (lessThanMax && moreThanBegin){
    armExtend.set(extendSpeed);
    } else if (retracting && moreThanBegin ||  !retracting && lessThanMax){
      armExtend.set(extendSpeed);
    } 
    System.out.println("lessthatnmax"+lessThanMax);
    System.out.println("retrancting "+retracting);
    System.out.println("retranctingspeed "+extendSpeed);
  }
  public void OverrideExtend(boolean t){
    extendOverride = t;
  }
   
}

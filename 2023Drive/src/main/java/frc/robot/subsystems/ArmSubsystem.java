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
import com.revrobotics.RelativeEncoder;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private TimeOfFlight armDistance;
  private boolean extendOverride;

  private RelativeEncoder ArmEncoder;
  private CANSparkMax armExtend;
  private CANSparkMax armRotate;
  private double angle;

  public ArmSubsystem() {
    this.armDistance = new TimeOfFlight(Constants.TIMEOFFLIGHT_ID);
    this.armExtend = new CANSparkMax(6, MotorType.kBrushless);
    this.armRotate = new CANSparkMax(5, MotorType.kBrushless);

    this.angle = 0.0; // Zero angle
    this.ArmEncoder.setPosition(0); // Zero encoders
    this.ArmEncoder = this.armRotate.getEncoder();
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
    //System.out.println("lessthatnmax"+lessThanMax);
    //System.out.println("retrancting "+retracting);
    System.out.println("retranctingspeed "+extendSpeed);
  }
  public void OverrideExtend(boolean t){
    extendOverride = t;
  }
  public boolean checkRotateDistance() {
    this.angle = (Math.abs(this.ArmEncoder.getPosition())) / 65.0;
    if (this.angle < 90) {
      return true;
    }
    return false;
  }
  public void rotateBack() {
    if(checkRotateDistance()) {
      this.armRotate.set(-1);
    }
  }
  public void rotateForward() {
    if(checkRotateDistance()) {
      this.armRotate.set(1);
    }
  }
   
}

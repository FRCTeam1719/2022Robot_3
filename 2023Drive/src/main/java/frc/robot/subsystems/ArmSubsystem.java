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
  private boolean rotateOverride;
  private RelativeEncoder ArmEncoder;
  private CANSparkMax armExtend;
  private CANSparkMax armRotate;
  private double angle;

  public ArmSubsystem() {
    this.armDistance = new TimeOfFlight(Constants.TIMEOFFLIGHT_ID);
    this.armRotate = new CANSparkMax(Constants.ARM_ROTATE_CAN_ID, MotorType.kBrushless);
    this.armExtend = new CANSparkMax(Constants.ARM_EXTEND_CAN_ID, MotorType.kBrushless);

    this.angle = 0.0; // Zero angle
    // TODO: wouldn't O.0 be if the arm was pointed straight up? I don't think that's a good idea

    this.ArmEncoder = this.armRotate.getEncoder();
    this.ArmEncoder.setPosition(0); // Zero encoders
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
      this.armExtend.set(extendSpeed);
    } else if (retracting && moreThanBegin ||  !retracting && lessThanMax){
      this.armExtend.set(extendSpeed);
    } 
  }
  public void OverrideExtend(boolean t){
    this.extendOverride = t;
  }
  public double checkRotateDistance() {
    this.angle = (this.ArmEncoder.getPosition()) / 65.0;
    return this.angle;
  }
 
  public void rotate(double rotateSpeed){
    boolean lessThanMax = this.checkRotateDistance() < 90;
    boolean moreThanBegin = this.checkRotateDistance() > -90;
    boolean retracting = rotateSpeed < 0;
    if (this.rotateOverride){
      this.armRotate.set(rotateSpeed);
    } else if (retracting && moreThanBegin ||  !retracting && lessThanMax){
      this.armRotate.set(rotateSpeed);
    } 
  }
  public void OverrideRotate(boolean t){
    this.rotateOverride = t;
  }
   
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
//import java.util.Timer;



public class ShooterSubsystem extends SubsystemBase {
    private static final double QUEUE_MOTOR_POWER = .5;
    private CANSparkMax m_shooterMotor1;  
    private CANSparkMax m_queueMotor1; 
    private CANSparkMax m_queueMotor2; 
    private double m_velocity = 1.0;

    private boolean m_isShooterOn = false;
    private boolean m_isQueue1On = false;
    private boolean m_isQueue2On = false;

  public ShooterSubsystem() {
    this.init();
  }
  
  private void init(){
    m_shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
    m_shooterMotor1.restoreFactoryDefaults();
    
    
    m_queueMotor1 = new CANSparkMax(Constants.QUEUE_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_queueMotor2 = new CANSparkMax(Constants.QUEUE_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_queueMotor1.restoreFactoryDefaults();
    m_queueMotor2.restoreFactoryDefaults();

    m_queueMotor2.setIdleMode(IdleMode.kBrake);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void increaseVelocity(double inc) {
    this.m_velocity = Math.min(1, this.m_velocity+inc);
    // this increases the speed of the shooter motor
  }

  public void increaseVelocity() {
    this.increaseVelocity(.05);
    // see above, by 0.5
  }

  public void setSpeed(double velocity) {
    this.m_velocity = velocity;
    // this is used to set the velocity to the required value
  }

  public double getVelocity() {
    return this.m_velocity;
    // this returns the current velocity
  }

  public void startShooter(){
    System.out.println("startShooter");
    m_shooterMotor1.set(this.m_velocity);  
    m_isShooterOn = true;  
      

// this shoots with a speed based on the velocity
  }
  public void stopShooter(){
    System.out.println("stopShooter");
    m_shooterMotor1.set(0);  
    m_isShooterOn = false;
  }

  public void  startQueueBelt(){
    System.out.println("startQueue");
    m_queueMotor1.set(1);  
   m_isQueue1On = true;
  }

  
  public void  stopQueueBelt(){
    System.out.println("stopQueue");
    m_queueMotor1.set(0); 
    m_isQueue1On = false;
    // this stops bringing balls upward
  }

  //new stuff
  public void  stopQueueFeederWheel(){
    System.out.println("stopQueue2");
    m_queueMotor2.set(0); 
    m_isQueue2On = false;
    //this stops the entrance to the shooter to allow shooter to be turned on
  }
  public void  startQueueFeederWheel(){
    System.out.println("startQueue2");
    m_queueMotor2.set(ShooterSubsystem.QUEUE_MOTOR_POWER); 
    m_isQueue2On = true;


    // this activates the queue motor that feeds balls into the shooter
  }

  public void toggleShooter(){ 
    if(m_isShooterOn == false){
      startShooter();
    }
    else{
      stopShooter(); 
    }
  }

  public void toggleQueue(){ 
    if(m_isQueue1On == false){
      startQueueBelt();
    }
    else{
      stopQueueBelt(); 
    }
  }

  public void toggleQueue2(){ 
    if(m_isQueue2On == false){
      startQueueFeederWheel();
    }
    else{
      stopQueueFeederWheel(); 
    }
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.lang.Math;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class MecanumDriveSubsystem extends SubsystemBase {

  private MecanumDrive m_myRobot;
  private CANSparkMax m_leftMotorFront;
  private RelativeEncoder m_leftFrontEncoder;
  private CANSparkMax m_rightMotorFront;
  private RelativeEncoder m_rightFrontEncoder;
  private CANSparkMax m_leftMotorBack;
  private RelativeEncoder m_leftBackEncoder;
  private CANSparkMax m_rightMotorBack;
  private RelativeEncoder m_rightBackEncoder;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("DriveTrainSubsystem");
  public final static double highSpeedLimit = .65;
  public final static double lowSpeedLimit = .50;
  private double currentSpeed = highSpeedLimit;

  private IdleMode m_idleMode = IdleMode.kBrake;
  public boolean doLog = false;
  private AHRS gyro;

  /** Creates a new DriveTrain. */
  public MecanumDriveSubsystem() {
    this.init();
  }

  public void setIdleMode(IdleMode idleMode) {
    this.m_idleMode = idleMode;
m_leftMotorFront.setIdleMode(this.m_idleMode);
   m_leftMotorBack.setIdleMode(this.m_idleMode);
   m_rightMotorFront.setIdleMode(this.m_idleMode);
   m_rightMotorBack.setIdleMode(this.m_idleMode);

  }

  public IdleMode getIdleMode() {
    return this.m_idleMode;
  }

  private void init() {

    gyro = new AHRS(SPI.Port.kMXP);
    
    m_leftMotorFront = new CANSparkMax(Constants.LEFT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_leftMotorBack = new CANSparkMax(Constants.LEFT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_leftMotorFront.setInverted(true);
    m_leftMotorBack.setInverted(true);

    m_rightMotorFront = new CANSparkMax(Constants.RIGHT_MOTOR_CAN1_ID, MotorType.kBrushless);
    m_rightMotorBack = new CANSparkMax(Constants.RIGHT_MOTOR_CAN2_ID, MotorType.kBrushless);
    m_rightMotorFront.restoreFactoryDefaults();
    m_rightMotorBack.restoreFactoryDefaults();

    m_leftFrontEncoder = this.m_leftMotorFront.getEncoder();
    m_rightFrontEncoder = this.m_rightMotorFront.getEncoder();
    m_leftBackEncoder =  this.m_leftMotorBack.getEncoder();
    m_rightBackEncoder = this.m_rightMotorBack.getEncoder();
resetGyro();
    zeroEncoders();
 
    this.setIdleMode(IdleMode.kCoast);
    m_myRobot = new MecanumDrive(m_leftMotorFront, m_leftMotorBack, m_rightMotorFront, m_rightMotorBack);
  }
  private void resetGyro(){
    gyro.resetDisplacement();
    gyro.zeroYaw();
  }
  public double getPitch(){
    return this.gyro.getPitch();
  }
  public void setSpeed(double speed) {
    this.currentSpeed = speed;
  }

  public void setMode(IdleMode mode) {
    this.setIdleMode(mode);
  }

  @Override
  public void periodic() {

  }
  

  public void MecanumDrive(
      double leftJoystickValueY,
      double leftJoystickValueX,
      double rightJoystickValueX) {

    if (doLog)
      System.out.println("joystick-X-Y,  " + leftJoystickValueX + ", " + leftJoystickValueY);

    // limit input to [-1,1] range
    leftJoystickValueX = -Math.pow(
        Math.signum(leftJoystickValueX) * Math.min(1, Math.abs(leftJoystickValueX)) * Constants.SPEED_REGULATOR, 3);
    leftJoystickValueY = Math.pow(
        Math.signum(leftJoystickValueY) * Math.min(1, Math.abs(leftJoystickValueY)) * Constants.SPEED_REGULATOR, 3);

    rightJoystickValueX = Math.pow(
        Math.signum(rightJoystickValueX) * Math.min(1, Math.abs(rightJoystickValueX)) * Constants.TURN_REGULATOR, 3);

    if (Math.abs(leftJoystickValueX) <= Constants.DEAD_ZONE_VALUE) {
      leftJoystickValueX = 0;
    }
    if (Math.abs(leftJoystickValueY) <= Constants.DEAD_ZONE_VALUE) {
      leftJoystickValueY = 0;
    }
    if (Math.abs(rightJoystickValueX) <= Constants.DEAD_ZONE_VALUE) {
      rightJoystickValueX = 0;
    }

     Rotation2d gyroAngle = this.gyro.getRotation2d();
     
    m_myRobot.driveCartesian(leftJoystickValueX, leftJoystickValueY, rightJoystickValueX 
    // , gyroAngle
    );
    // m_myRobot.driveCartesian(0.1, 0,0);
  }

public void testAutondrive(double lx,double ly,double rx){
  
  lx =Math.signum(lx) * Math.min(1, Math.abs(lx));
  ly =Math.signum(ly) * Math.min(1, Math.abs(ly));
  rx =Math.signum(rx) * Math.min(1, Math.abs(rx));
   
  
  m_myRobot.driveCartesian(lx, ly, rx) ;
}

  public void MecanumPolarDrive(double Speed, Rotation2d Angle, double rotate){

    Speed = -Math.pow(
      Math.signum(Speed) * Math.min(1, Math.abs(Speed)) * Constants.SPEED_REGULATOR, 3);


  rotate = Math.pow(
      Math.signum(rotate) * Math.min(1, Math.abs(rotate)) * Constants.TURN_REGULATOR, 3);

  if (Math.abs(Speed) <= Constants.DEAD_ZONE_VALUE) {
    Speed = 0;
  }
  
  if (Math.abs(rotate) <= Constants.DEAD_ZONE_VALUE) {
    rotate = 0;
  }
  Rotation2d gyroAngle = this.gyro.getRotation2d();
  Rotation2d DriveAngle = new Rotation2d(gyroAngle.getDegrees() + Angle.getDegrees());
  m_myRobot.drivePolar(Speed, DriveAngle, rotate  );

  }
public double getGyroDispY(){
  return this.gyro.getDisplacementY();
}
//I read this, and maybe the encoders will be more accurate.
// Returns the displacement (in meters) of the Y axis since resetDisplacement() was last invoked [Experimental]. NOTE: This feature is experimental. Displacement measures rely on double-integration of acceleration values from MEMS accelerometers which yield "noisy" values. The resulting displacement are not known to be very accurate, and the amount of error increases quickly as time progresses.
public double getGyroDispX(){
  return this.gyro.getDisplacementX();
}

  public double getLeftFrontPosition() {
    SmartDashboard.putNumber("m_leftFrontEncoder", this.m_leftFrontEncoder.getPosition());
    return this.m_leftFrontEncoder.getPosition();
  }

  private double getLeftBackPosition() {
    return this.m_leftBackEncoder.getPosition();
  }
  
  private double getRightFrontPosition() {
    return this.m_rightFrontEncoder.getPosition();
  }

  private double getRightBackPosition() {
    return this.m_rightBackEncoder.getPosition();
  }

  public void zeroEncoders() {
    this.m_leftFrontEncoder.setPosition(0);
    this.m_leftBackEncoder.setPosition(0);
    this.m_rightFrontEncoder.setPosition(0);
    this.m_rightBackEncoder.setPosition(0);
  }

}
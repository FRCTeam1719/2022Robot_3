// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSequentialCommands extends SequentialCommandGroup {
  private MecanumDriveSubsystem m_DriveSubsystem;
private GrabberSubsystem m_grabberSubsystem;
private ArmSubsystem m_armSubsystem;
private Rotation2d dAngle;
private double dispY=0;
private double dispX=0;
private double setpoint = 0;
  /** Creates a new AutonSequentialCommands. */
  public AutonSequentialCommands(MecanumDriveSubsystem DriveSubsystem, GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem) {
    this.m_DriveSubsystem = DriveSubsystem;
    this.m_grabberSubsystem = grabberSubsystem;
    this.m_armSubsystem = armSubsystem;
    dispY=0;
    dispX=0;
    setpoint = 0;
    testAutongrab();
    //testAutondrive();

     
  }
  
  private void testAutongrab(){
      // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    
  //isGrab(true), 
zerCommand(),
forward(5),
isGrab(true),
delay(2),
isGrab(false)


  

  
  // ...

  );
  }
  private void testAutondrive(){
    // Addd your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());
  addCommands( 
    zerCommand(),
    setTarget(10),
    checkTarget()
   
    
  

);
}
  private void addCommands(InstantCommand zerCommand, Object println, InstantCommand setTarget,
      InstantCommand checkTarget) {
  }

  private InstantCommand isGrab(boolean state){
    return new InstantCommand(() ->{ this.m_grabberSubsystem.Grab(state);});
  }

  private InstantCommand checkTarget(){
    if (this.m_DriveSubsystem.getLeftFrontPosition() < setpoint ) {
      Drive();
      SmartDashboard.putBoolean("zerCommand", true);
      return checkTarget();
      
    } else {
      return stopDrive();
    }
    
    
  }
  private InstantCommand setTarget(double target){
return new InstantCommand(()-> {setpoint = target;});
  }
  private InstantCommand zerCommand(){
    return new InstantCommand(() ->{ this.m_DriveSubsystem.zeroEncoders();});
  }
  private InstantCommand Drive(){
    return new InstantCommand(() ->{ this.m_DriveSubsystem.MecanumDrive(0, 1, 0);});

  }  private InstantCommand stopDrive(){
    return new InstantCommand(() ->{ this.m_DriveSubsystem.MecanumDrive(0, 0, 0);});
  }
  private WaitCommand delay(double s){
    return new WaitCommand(s);
  }
  private PIDextendArmCommand armTarget(double target){
    return new PIDextendArmCommand(target, this.m_armSubsystem);

  }
  private PIDbalancerCommand balance(){
    return new PIDbalancerCommand(this.m_DriveSubsystem);
  }
  private PIDforwardCommand forward(double distance){
    dispY+=distance;
    SmartDashboard.putNumber("dispY", dispY);
    
    return new PIDforwardCommand(this.m_DriveSubsystem, dispY);
  }

  //negative values should go left
  private PIDsidewaysCommand right(double distance){
    dispX-=distance;
    
    return new PIDsidewaysCommand(this.m_DriveSubsystem, dispX);
  }
}


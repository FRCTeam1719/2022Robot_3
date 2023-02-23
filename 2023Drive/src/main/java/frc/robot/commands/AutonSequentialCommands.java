// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSequentialCommands extends SequentialCommandGroup {
  private MecanumDriveSubsystem DriveSubsystem;
private GrabberSubsystem grabberSubsystem;
private ArmSubsystem armSubsystem;
private Rotation2d dAngle;
private double dispY;
  /** Creates a new AutonSequentialCommands. */
  public AutonSequentialCommands(MecanumDriveSubsystem DriveSubsystem, GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem) {

    
    testAuton();

     
  }
  
  private void testAuton(){
      // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    
  isGrab(true), 
  delay(4),
  isGrab(false) 
  

  
  // ...

  );
  }
  private InstantCommand isGrab(boolean state){
    return new InstantCommand(() ->{ grabberSubsystem.Grab(state);});
  }
  private WaitCommand delay(double s){
    return new WaitCommand(s);
  }
  private PIDextendArmCommand armTarget(double target){
    return new PIDextendArmCommand(target, armSubsystem);

  }
  private PIDbalancerCommand balance(){
    return new PIDbalancerCommand(DriveSubsystem);
  }
  private PIDforwardCommand forward(double distance){
    dispY+=distance;
    
    return new PIDforwardCommand(DriveSubsystem, dispY);
  }
}


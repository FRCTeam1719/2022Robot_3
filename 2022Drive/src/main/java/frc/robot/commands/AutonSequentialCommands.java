// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSequentialCommands extends SequentialCommandGroup {
  /** Creates a new AutonSequentialCommands. */
  private DriveTrainSubsystem m_tankDriveSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private BeltSubsystem m_beltSubsystem;
  private LimelightVisionSubsystem m_limelightVisionSubsystem;
  private double feetTotal = 0;
  private double angleTotal = 0;
  public AutonSequentialCommands(
      DriveTrainSubsystem tankDriveSubsystem
      , IntakeSubsystem intakeSubsystem
      , ShooterSubsystem shooterSubsystem
      , LimelightVisionSubsystem limelightVisionSubsystem
      , BeltSubsystem beltSubsystem) {
    m_limelightVisionSubsystem = limelightVisionSubsystem;
    m_tankDriveSubsystem = tankDriveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beltSubsystem = beltSubsystem;
    addRequirements(m_tankDriveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_limelightVisionSubsystem, m_beltSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    
    addCommands(
      // all commands will go here with commas after them.
      drive(1.5), // move for the first shot
      shootSequence(),  //shoot first shot
      turn(180),  // turn around to pick up the second ball
      drive(2),   // pick up the second ball //TODO: do we need start the intake?
      turn(180),  // turn to shoot the second ball // do we need to drive back to the original position
      shootSequence()  // second shot
    );
  }

  private DriveDistancePidCommand  drive(double feet){
    feetTotal += feet;
    System.out.println(feetTotal);
    return new DriveDistancePidCommand(m_tankDriveSubsystem, feetTotal);
  }
<<<<<<< HEAD

  public TurnAnglePidCommand  turn(double angle){
=======
  
  private TurnAnglePidCommand  turn(double angle){
>>>>>>> main
    angleTotal += angle;
    System.out.println(angleTotal);
    return new TurnAnglePidCommand(m_tankDriveSubsystem, angleTotal);
  }

  // public ShootCommands shootSequence5(){
  //   return new ShootCommands(m_shooterSubsystem);
  // }

  public FollowLimelightPidCommand followlimelight(){
    return new FollowLimelightPidCommand(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public FollowLimelightSequence limelightSequence(){
    return new FollowLimelightSequence(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public InstantCommand TurnLimelightOn(){
    return new InstantCommand(
      ()->{
<<<<<<< HEAD
        System.out.println("limelight start");
        m_limelightVisionSubsystem.turnOnLed();
      });
  }

  public InstantCommand TurnLimelightOff(){
    return new InstantCommand(
      ()->{
=======
>>>>>>> main
       System.out.println("limelight stop");
       m_limelightVisionSubsystem.turnOffLed();
        // the following lines will be removed later
        m_tankDriveSubsystem.zeroEncoders();
       feetTotal=0;
       angleTotal=0;
    });
    }

    public ShootSequence shootSequence(){
      return new ShootSequence(this.m_shooterSubsystem, this.m_beltSubsystem);
    }
    public InstantCommand intakePull(){
      return new InstantCommand(
        ()->{
      this.m_intakeSubsystem.intakePull();
      }, this.m_intakeSubsystem, this.m_beltSubsystem);
    }
    public InstantCommand intakeStop(){
      return new InstantCommand(
        ()->{
      this.m_intakeSubsystem.intakeStop();
      }, this.m_intakeSubsystem, this.m_beltSubsystem);
    }

    public FollowLimelightSequence followlimelightsequence(){
     return new FollowLimelightSequence(m_tankDriveSubsystem, m_limelightVisionSubsystem);
    }
    public SequentialCommandGroup goback(){
      return turn(0).andThen(drive(0)).andThen(turn(0));
    }
    public SequentialCommandGroup shootWithLimelight(){
      return followlimelightsequence().andThen(goback());
    }
}



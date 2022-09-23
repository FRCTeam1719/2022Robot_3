// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Queue;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LinearSetpointTrajectory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.QueueFeederWheelSubsystem;
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
  private QueueFeederWheelSubsystem m_queueFeederWheelSubsystem;
  private ClimberSubsystem m_leftClimberSubsystem;
  private ClimberSubsystem m_rightClimberSubsystem;
  


  private double feetTotal = 0;
  private double angleTotal = 0;
  public AutonSequentialCommands(
      DriveTrainSubsystem tankDriveSubsystem
      , IntakeSubsystem intakeSubsystem
      , ShooterSubsystem shooterSubsystem
      , LimelightVisionSubsystem limelightVisionSubsystem
      , BeltSubsystem beltSubsystem
      , QueueFeederWheelSubsystem queueFeederWheelSubsystem
      , ClimberSubsystem leftClimberSubsystem
      , ClimberSubsystem rightClimberSubsystem
      
      ) {
    m_limelightVisionSubsystem = limelightVisionSubsystem;
    m_tankDriveSubsystem = tankDriveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_beltSubsystem = beltSubsystem;
    this.m_queueFeederWheelSubsystem = queueFeederWheelSubsystem;
    this.m_leftClimberSubsystem = leftClimberSubsystem;
    this.m_rightClimberSubsystem = rightClimberSubsystem;

    addRequirements(m_tankDriveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_limelightVisionSubsystem, m_beltSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    
    addCommands(

        // new InstantCommand(()->this.m_beltSubsystem.startBelt(1))
        // , new WaitCommand(10)
        // ,new InstantCommand(()->this.m_beltSubsystem.stopBelt())
        

      // all commands will go here with commas after them.
      new InstantCommand( ()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kBrake))
      , shootSequence()
      , drive(2)
      , turn(180)
      , new InstantCommand(() -> this.m_intakeSubsystem.intakePull())
      , drive(-4)
      , turn (180)
      , drive (6.5)
      , new InstantCommand(() -> this.m_intakeSubsystem.intakeStop())
      , shootSequence()
  
      // , turn(180)
      // //, drive(2)
      // , new WaitCommand(.3)
      // , turn(30)
      // , new WaitCommand(.3)
      // , turn(-30)
      // , new WaitCommand(.3)
      // , turn(30)
      // , new WaitCommand(.3)
      // , turn(-30)
      // , new WaitCommand(.3)
      // , new ParallelCommandGroup(
      //      turn(360)
      
      //      , 
      //      new SequentialCommandGroup(           
      //        MoveClimberCommand(this.m_rightClimberSubsystem, 7, .001),
      //       new ParallelCommandGroup(
      //             MoveClimberCommand(this.m_rightClimberSubsystem, 0, .001),
      //             MoveClimberCommand(this.m_leftClimberSubsystem, 7, .001)
      //           )
      //       , new WaitCommand(.3)
      //       ,new ParallelCommandGroup(
      //             MoveClimberCommand(this.m_rightClimberSubsystem, 7, .001),
      //             MoveClimberCommand(this.m_leftClimberSubsystem, 0, .001)
      //           )
      //       , new WaitCommand(.3)
      //       ,new ParallelCommandGroup(
      //             MoveClimberCommand(this.m_rightClimberSubsystem, 0, .001),
      //             MoveClimberCommand(this.m_leftClimberSubsystem, 7, .001)
      //           )
      //       , new WaitCommand(.3)
            
      //       ,new ParallelCommandGroup(
      //         MoveClimberCommand(this.m_rightClimberSubsystem, 7, .001),
      //         MoveClimberCommand(this.m_leftClimberSubsystem, 0, .001)
      //       )
      //       , new WaitCommand(.3)
      //       ,new ParallelCommandGroup(
      //             MoveClimberCommand(this.m_rightClimberSubsystem, 0, .001),
      //             MoveClimberCommand(this.m_leftClimberSubsystem, 7, .001)
      //           )
            
      //      )
      // )
      // , new ParallelCommandGroup(
      //   MoveClimberCommand(this.m_rightClimberSubsystem, 0, .001),
      //   MoveClimberCommand(this.m_leftClimberSubsystem, 0, .001)
      // )
      //, new InstantCommand( this.m_leftClimberSubsystem.)
      //, new InstantCommand(this)
      // , new InstantCommand(() -> this.m_intakeSubsystem.intakePull())
      // , drive(-5)
      // , new WaitCommand(.5)   // pick up the second ball 
      // , new InstantCommand( ()-> this.m_intakeSubsystem.intakeStop())
      // ,turn(180)
      // , drive(-7)
      // , new InstantCommand(()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kCoast))
      // , shootSequence()  // second shot
      
    
    );
  }
private static Command MoveClimberCommand(ClimberSubsystem climberSubsystem, 
double positionTarget, double moveTimeInMilliseconds) {
  return new PIDClimbCommand(
          climberSubsystem,                //subsystem
          new LinearSetpointTrajectory(
            climberSubsystem.getPosition(), 
            positionTarget, moveTimeInMilliseconds, "rightClimber"),
          1,                                     // climb speed
          true,                                   // does it end (otherwise keep holding)
          "right",
          RobotContainer.getExtendClimberPidSettings());
        
}
// NOTE: this code is never used, see yellow line
  private void getFirstAuton() {
    addCommands(
      // all commands will go here with commas after them.
      new InstantCommand( ()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kBrake))
      , drive(2.0) // move for the first shot
      , shootSequence() //shoot first shot
      , turn(180)  // turn around to pick up the second ball
      , new WaitCommand(1)
      , new InstantCommand(() -> this.m_intakeSubsystem.intakePull())
      , new WaitCommand(.5)
      , drive(-2.0)   // pick up the second ball //TODO: do we need start the intake?
      , new WaitCommand(.5)
      , new InstantCommand( ()-> this.m_intakeSubsystem.intakeStop())
      , turn(180)  // turn to shoot the second ball // do we need to drive back to the original position
      , drive(-2.0)
      , new WaitCommand(.5)
      , shootSequence()  // second shot
      , new InstantCommand(()-> this.m_tankDriveSubsystem.setIdleMode(IdleMode.kCoast))
    );
  }

  private DriveDistancePidCommand drive(double feet){
    feetTotal += feet;
    //System.out.println(feetTotal);
    return new DriveDistancePidCommand(m_tankDriveSubsystem, feetTotal);
  }

  public TurnAnglePidCommand  turn(double angle){
    angleTotal += angle;
    //System.out.println(angleTotal);
    return new TurnAnglePidCommand(m_tankDriveSubsystem, angleTotal);
  }


  public FollowLimelightPidCommand followlimelight(){
    return new FollowLimelightPidCommand(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public FollowLimelightSequence limelightSequence(){
    return new FollowLimelightSequence(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem);
  }
    
  public InstantCommand TurnLimelightOn(){
    return new InstantCommand(
      ()->{
        //System.out.println("limelight start");
        m_limelightVisionSubsystem.turnOnLed();
      });
  }

  public InstantCommand TurnLimelightOff(){
    return new InstantCommand(
      ()->{
       //System.out.println("limelight stop");
       m_limelightVisionSubsystem.turnOffLed();
        // the following lines will be removed later
        m_tankDriveSubsystem.zeroEncoders();
       feetTotal=0;
       angleTotal=0;
    });
    }

    public ShootSequence shootSequence(){
      return new ShootSequence(
        this.m_shooterSubsystem, this.m_beltSubsystem,
        this.m_queueFeederWheelSubsystem);
    }
    public InstantCommand intakePull(){
      return new InstantCommand(
        ()->{
      this.m_intakeSubsystem.intakePull();
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



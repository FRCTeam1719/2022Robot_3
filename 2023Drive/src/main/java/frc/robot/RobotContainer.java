// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methFods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum Mode {
    Undefined,
    TeleOp,
    Test,
    Autonomous,
    Practice
  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("RobotContainer");

  XboxController m_driveController = new XboxController(Constants.DRIVE_XBOX_CONTROLLER);
  XboxController m_helperController = new XboxController(Constants.HELPER_XBOX_CONTROLLER);

  // The robot's subsystems and commands are defined here...
  private final MecanumDriveSubsystem m_MecanumDriveSubsystem = new MecanumDriveSubsystem();
  // private final DriveTrainSubsystem driveSubsystem = new
  // DriveTrainSubsystem();;
  private final GrabberSubsystem Grabber = new GrabberSubsystem();
  private final BreakSubsystem m_Break = new BreakSubsystem();
  //private final ArmSubsystem m_Arm = new ArmSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private edu.wpi.first.wpilibj2.command.button.Trigger whenPressed;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    table.getEntry("isXboxConnected").setBoolean(m_helperController.isConnected());

    MecanumDriveCommand DriveMode = new MecanumDriveCommand(
        this.m_MecanumDriveSubsystem, m_driveController::getLeftY, m_driveController::getLeftX,
        m_driveController::getRightX

    );
    this.m_MecanumDriveSubsystem.setDefaultCommand(DriveMode);
    new JoystickButton(m_helperController, Button.kB.value)
        .onTrue(new InstantCommand(() -> {
          System.out.println(this.limelight.getDistance());
          new MecanumPIDCommand(this.limelight, this.m_MecanumDriveSubsystem);
        }));
    new JoystickButton(m_driveController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
          this.m_MecanumDriveSubsystem.setMode(IdleMode.kCoast);
        }));
    new JoystickButton(m_driveController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> {
          this.m_MecanumDriveSubsystem.setMode(IdleMode.kBrake);
        }));
    new JoystickButton(m_driveController, Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          this.m_Break.ToggleBreak();
        }));
    // new JoystickButton(m_driveController, Button.kA.value)
    //     .onTrue(new InstantCommand(() -> {
    //       this.m_Arm.getArmDistance();
    //     }));
  }

  public Command getAutonomousCommand() {
    return null;
  }

}

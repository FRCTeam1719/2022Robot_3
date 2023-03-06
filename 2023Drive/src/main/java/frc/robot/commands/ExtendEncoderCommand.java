// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendEncoderCommand extends PIDCommand {
  /** Creates a new ExtendEncoderCommand. */
  public ExtendEncoderCommand(ArmSubsystem Arm, double percentTarget) {
    super(
        // The controller that the command will use
        new PIDController(1, 0.1, 0.1),
        // This should return the measurement
        () -> Arm.getArmEncoderDistance(),
        // This should return the setpoint (can also be a constant)
        () -> percentTarget,
        // This uses the output
        output -> {
          Arm.extend(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1.0);
    SmartDashboard.putBoolean("Talererance",true );  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

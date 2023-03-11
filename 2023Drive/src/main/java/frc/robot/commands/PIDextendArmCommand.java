// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDextendArmCommand extends PIDCommand {
  /** Creates a new PIDextendArmCommand. */
  public PIDextendArmCommand(double target, ArmSubsystem armSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(.2, 0, 0),
        // This should return the measurement
        () -> armSubsystem.getArmEncoderDistance(),
        // This should return the setpoint (can also be a constant)
        () -> target,
        // This uses the output
        output -> {
          
          armSubsystem.extend(output);
          SmartDashboard.putBoolean("PID extendarm command",true );
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1, 0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

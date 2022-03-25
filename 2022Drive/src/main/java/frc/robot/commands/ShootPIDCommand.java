// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.LinearSetpointTrajectory;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPIDCommand extends PIDCommand {
  /** Creates a new ShootPIDCommand. */
  public ShootPIDCommand(LinearSetpointTrajectory setpointTrajectory, ShooterSubsystem m_shooterSubsystem) {
    super(
        // The controller that the command will use
        //TODO: avoid hardcoding
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> m_shooterSubsystem.getVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> setpointTrajectory.getSetpoint(),
        // This uses the output
        output -> {
          //TODO: this is unlikely to work - the output is not the increase.  it the value to set it to.  
          m_shooterSubsystem.increaseVelocity(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

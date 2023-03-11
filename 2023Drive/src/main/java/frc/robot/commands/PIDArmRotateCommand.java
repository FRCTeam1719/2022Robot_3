// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDArmRotateCommand extends PIDCommand {
  /** Creates a new PIDArmRotateCommand. */
  public PIDArmRotateCommand(ArmSubsystem arm, double target) {
    super(
        // The controller that the command will use
        new PIDController(5, 0, .925),
        // This should return the measurement
        () -> arm.rotateAngle(),
        // This should return the setpoint (can also be a constant)
        target,
        // This uses the output
        output -> {
          // Use the output here
          arm.rotate(output);
        });
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(arm);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.005, 0.005);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

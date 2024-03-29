// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.MecanumDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDbalancerCommand extends PIDCommand {
  /** Creates a new PIDbalancerCommand. */
  public PIDbalancerCommand(MecanumDriveSubsystem balance) {
    super(
        // The controller that the command will use
        new PIDController( Constants.DRIVEkp, Constants.DRIVEki, Constants.DRIVEkd),
        // This should return the measurement
        () -> balance.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          balance.MecanumDrive(output, 0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(balance);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

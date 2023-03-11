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
public class PIDforwardCommand extends PIDCommand {
 // Creates a new PIDforwardCommand. 
  public PIDforwardCommand(MecanumDriveSubsystem DriveSubsystem, double target) {
    super(
        // The controller that the command will use
        new PIDController( Constants.DRIVEkp, Constants.DRIVEki, Constants.DRIVEkd),
        // This should return the measurement
        () -> DriveSubsystem.getLeftFrontPosition(),
        // This should return the setpoint (can also be a constant)
        target,
        // This uses the output
        output -> {
          DriveSubsystem.testAutondrive(output, 0, 0);
          System.out.print(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1,1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

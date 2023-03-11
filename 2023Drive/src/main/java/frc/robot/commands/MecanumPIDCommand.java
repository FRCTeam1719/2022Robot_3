// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MecanumPIDCommand extends PIDCommand {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("mecanumPID");
  /** Creates a new MecanumPIDCommand. */
  public double DistanceFromGoal;

  public MecanumPIDCommand(LimelightSubsystem limelight, MecanumDriveSubsystem driver) {
    super(
        // The controller that the command will use
        new PIDController( Constants.DRIVEkp, Constants.DRIVEki, Constants.DRIVEkd),
        // This should return the measurement
        () -> limelight.getDistance(),
        // This should return the setpoint (can also be a constant)
        Constants.INCHES_FROM_GOAL,
        // This uses the output
        output -> {
          // change other 2 values later
          //System.out.println(output);
          driver.MecanumDrive(output, 0, 0);
        });

    getController().setTolerance(4,0.5);
    addRequirements(driver);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

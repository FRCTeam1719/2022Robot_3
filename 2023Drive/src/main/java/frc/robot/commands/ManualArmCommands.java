// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommands extends CommandBase {
  /** Creates a new ManualArmCommands. */
  private DoubleSupplier leftY;
  private DoubleSupplier rightY;
  private ArmSubsystem arm;

  public ManualArmCommands(ArmSubsystem m_arm, DoubleSupplier m_leftY, DoubleSupplier m_rightY) {
    addRequirements(m_arm);

    this.leftY = m_leftY;
    this.rightY = m_rightY;
    this.arm = m_arm;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.arm.extend(-Math.pow(
        Math.signum(this.rightY.getAsDouble()) * Math.min(1, Math.abs(this.rightY.getAsDouble())) * Constants.ARMEXTEND_SCALER, 3));
    this.arm.rotate(-Math.pow(
        Math.signum(this.leftY.getAsDouble()) * Math.min(1, Math.abs(this.leftY.getAsDouble())) * Constants.ARMSPEED_SCALER, 3));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class ReverseConveyors extends CommandBase {
  /**
   * Creates a new ReverseConveyors.
   */
  TurretSubsystem m_turretSubsystem;
  public ReverseConveyors(TurretSubsystem turretSubsystem) {
    this.m_turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turretSubsystem.move_intake_motor(-1 * Constants.INTAKE_WHEEL_SPEED);
    m_turretSubsystem.move_lower_conveyor(-1 * Constants.LOWER_CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.move_intake_motor(0);
    m_turretSubsystem.move_lower_conveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevator extends CommandBase {
  /**
   * Creates a new MoveElevator.
   */

  ElevatorSubsystem m_elevatorSubsystem;
  double speed;
  double setpoint;

  public MoveElevator(ElevatorSubsystem elevatorSubsystem, double speed) {
    this.m_elevatorSubsystem = elevatorSubsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.moveElevatorOpenLoop(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // setpoint += setpointAdjustmentFactor * Constants.ELEVATOR_SETPOINT_SCALING;
    // m_elevatorSubsystem.moveElevatorSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.moveElevatorOpenLoop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;;

public class RobotClimb extends CommandBase {
  /**
   * Creates a new RobotClimb.
   */

<<<<<<< HEAD
  private final ElevatorPIDSubsystem m_elevatorSubsystem;
  int position_counts;

  public RobotClimb(ElevatorPIDSubsystem elevatorSubsystem, int position_counts) {
=======
  private final ElevatorSubsystem m_elevatorSubsystem;

  public RobotClimb(ElevatorSubsystem elevatorSubsystem) {
>>>>>>> f8b8a16de4a1729d0ed80b1c22640f261408a623
    this.m_elevatorSubsystem = elevatorSubsystem;
    this.position_counts = position_counts;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    m_elevatorSubsystem.setSetpoint(position_counts);
=======
    m_elevatorSubsystem.setPositionSetpoint(Constants.CLIMB_POSITION_COUNTS);
>>>>>>> f8b8a16de4a1729d0ed80b1c22640f261408a623
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.getError() < Math.abs(Constants.ELEVATOR_TOLERANCE);
  }
}

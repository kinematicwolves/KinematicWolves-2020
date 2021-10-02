package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ConveyorHorizontal extends CommandBase {
  /**
   * Creates a new ConveyorHorizontal.
   */
  TurretSubsystem m_turretSubsystem;
  double conveyorSpeed;
  public ConveyorHorizontal(TurretSubsystem turretSubsystem, double conveyorSpeed) {
    this.conveyorSpeed = conveyorSpeed;
    m_turretSubsystem = turretSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_turretSubsystem.move_lower_motor(intakeWheelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_turretSubsystem.move_lower_conveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

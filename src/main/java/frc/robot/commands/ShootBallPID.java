package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;;


public class ShootBallPID extends CommandBase {
  /**
   * Creates a new ShootBallPID.
   */


  // The subsystem the command runs on
  private final ShooterSubsystem m_shooterSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final TurretSubsystem m_turretSubsystem;
  double timer;

  public ShootBallPID(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turretSubsystem = turretSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_visionSubsystem = visionSubsystem;
    addRequirements(m_shooterSubsystem);
    addRequirements(m_visionSubsystem);
    this.timer = 0;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooterSubsystem.move_top_conveyor(Constants.UPPER_CONVEYOR_SPEED);
    m_turretSubsystem.override_Lower_conveyor(Constants.UPPER_CONVEYOR_SPEED);
    
    // double distance = m_visionSubsystem.getDistance(); // TODO: Units
    // double distance = 10; // feet
    // double speed = Utilities.linearInterpolation(Constants.distances, Constants.speeds, distance);
    timer = 0;
    int speed = 1000;  //rpm
    
    m_shooterSubsystem.setVelocity_rpm(speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20; // ms
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooterSubsystem.move_top_conveyor(0);
    m_shooterSubsystem.shootBall(0);
    m_turretSubsystem.move_lower_conveyor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2000;
  }
}
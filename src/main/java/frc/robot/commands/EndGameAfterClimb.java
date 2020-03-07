// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.ElevatorPIDSubsystem;

// public class EndGameAfterClimb extends CommandBase {
//   /**
//    * Creates a new EndGameAfterClimb.
//    */
//   private ElevatorPIDSubsystem m_elevatorSubsystem;
//   private double timeout;
//   double timer;

//   public EndGameAfterClimb(ElevatorPIDSubsystem elevatorSubsystem, double timeout) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.m_elevatorSubsystem = elevatorSubsystem;
//     this.timeout = timeout;
//     this.timer = 0;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_elevatorSubsystem.setSetpoint(Constants.ElevatorInitialPosition);
//   }


//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer += 20;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_elevatorSubsystem.setSetpoint(Constants.FINAL_ELEVATOR_POSITION);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer > timeout;
//   }
// }

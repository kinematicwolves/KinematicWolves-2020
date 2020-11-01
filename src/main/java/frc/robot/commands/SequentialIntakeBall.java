/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class SequentialIntakeBall extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public SequentialIntakeBall(ConveyorSubsystem turretSubsystem, double intakeWheelSpeed, double conveyorSpeed) {
    addCommands(
      new InitCommandUntilBallFound(turretSubsystem, conveyorSpeed),
      new IntakeBall(turretSubsystem, intakeWheelSpeed, conveyorSpeed),
      new DelayIntakeConveyorStop(turretSubsystem, conveyorSpeed)
    );
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

  }
}

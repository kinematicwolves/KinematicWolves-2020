/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_elevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_TALON_FX);

  double init_setpoint;
  double setpoint = 0;

  // private final ElevatorFeedforward m_elevatorFeedforward =
  //     new ElevatorFeedforward(Constants.kS,
  //                                Constants.kG, Constants.kV, Constants.kA);
  // Docs: https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/controller/ElevatorFeedforward.html

  /**

   * The Elevator subsystem for the robot.

   */

  public ElevatorSubsystem() {
    // configureFeedback();
    // this.init_setpoint = m_elevatorMotor.getSelectedSensorPosition(Constants.ELEVATOR_PID_LOOP);
    // this.setpoint += this.init_setpoint;
    m_elevatorMotor.setInverted(true);
    
  }
 
  private void configureFeedback(){
    m_elevatorMotor.configFactoryDefault();
    m_elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.ELEVATOR_PID_LOOP,
    Constants.ELEVATOR_kTIMEOUT);
    m_elevatorMotor.config_kF(Constants.ELEVATOR_PID_LOOP, Constants.ELEVATOR_kF);
    m_elevatorMotor.config_kP(Constants.ELEVATOR_PID_LOOP, Constants.ELEVATOR_kP);
    m_elevatorMotor.config_kI(Constants.ELEVATOR_PID_LOOP, Constants.ELEVATOR_kI);
    m_elevatorMotor.config_kD(Constants.ELEVATOR_PID_LOOP, Constants.ELEVATOR_kD);
    
    m_elevatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.ELEVATOR_kTIMEOUT);
  }

  // public void moveElevatorSetpoint(double setpointAdjustmentFactor){
  //   // Move elevator setpoint by setpointAdjustmentFactor
  //   this.setpoint += setpointAdjustmentFactor;
  //   // m_elevatorMotor.set(ControlMode.Position, checkElevatorSetpoint(this.setpoint));
  // }

  // public void setPositionSetpoint(double setpoint){
  //   this.setpoint = setpoint + init_setpoint;
  //   // m_elevatorMotor.set(ControlMode.Position, this.setpoint);
  // }

  // public double getPosition_counts(){
  //   return   m_elevatorMotor.getSelectedSensorPosition();
  // }

  // public double getError(){
  //   return m_elevatorMotor.getClosedLoopError();
  // }

  // public void stopElevator() {
  //   m_elevatorMotor.set(0);
  // }

  public void moveElevatorOpenLoop(double speed){
    m_elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  private double checkElevatorSetpoint(double setpoint){
    if (setpoint > Constants.MAX_ELEVATOR_HEIGHT){
      setpoint = Constants.MAX_ELEVATOR_HEIGHT;
    }
    
    if (setpoint < Constants.MIN_ELEVATOR_HEIGHT){
      setpoint = Constants.MIN_ELEVATOR_HEIGHT;
    }
    return setpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Elevator Position Counts", getPosition_counts());
    // SmartDashboard.putNumber("Elevator Error", getError());
    // SmartDashboard.putNumber("Elevator setpoint (counts)", this.setpoint);
  }
}
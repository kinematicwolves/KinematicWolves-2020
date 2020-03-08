/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */

  public static WPI_TalonSRX spinShooterTalon_1 = new WPI_TalonSRX(Constants.SPIN_SHOOTER_MOTOR_1); // This is the CAN ID for the device 
  public static WPI_TalonSRX spinShooterTalon_2 = new WPI_TalonSRX(Constants.SPIN_SHOOTER_MOTOR_2); // This is the CAN ID for the device 
  public static WPI_TalonSRX topConveyorTalon = new WPI_TalonSRX(Constants.TOP_CONVEYOR_MOTOR);
  public static Servo angleActuator_1 = new Servo(Constants.LINEAR_ACTUATOR_1); // PWM controlled
  public static Servo angleActuator_2 = new Servo(Constants.LINEAR_ACTUATOR_2); // PWM controlled

  // Need a static and velocity gain for the motors for below FF term, need to use setVoltage() method with it
  // public static SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ks, kv);
  
  public ShooterSubsystem(){
    spinShooterTalon_1.setInverted(true);
    spinShooterTalon_2.setInverted(true);
    spinShooterTalon_2.follow(spinShooterTalon_1);
    configFeedbackControl();

  }

  private void configFeedbackControl(){
    spinShooterTalon_1.config_kP(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_kP, Constants.SHOOTER_PID_TIMEOUT);
    spinShooterTalon_1.config_kI(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_kI, Constants.SHOOTER_PID_TIMEOUT);
    spinShooterTalon_1.config_kD(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_kD, Constants.SHOOTER_PID_TIMEOUT);
    spinShooterTalon_1.config_kF(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_kF, Constants.SHOOTER_PID_TIMEOUT);
  }

  private double getVelocity_rpm(){
    double speed_counts_per_sec = spinShooterTalon_1.getSelectedSensorVelocity(
      Constants.SHOOTER_PID_SLOT) / 0.1; // counts per 100 ms, convert to sec
    double speed_rpm = speed_counts_per_sec * 60 / Constants.SHOOTER_ENCODER_CPR;
    return speed_rpm;
  }

  public void setVelocity_rpm(int setpoint_rpm){
    double setpoint_counts_per_100ms = setpoint_rpm / 60 * Constants.SHOOTER_ENCODER_CPR;
    spinShooterTalon_1.set(ControlMode.Velocity, setpoint_counts_per_100ms);
  }

  public void shootBall(final double speed) {
    spinShooterTalon_1.set(speed);
    spinShooterTalon_2.set(speed);
  }

  public void setLinearActuatorPosition(double position) {
    // Check to make sure position is not out of bounds
    position = clipLinearActuatorPositionCommand(position);
    angleActuator_1.set(position);
    angleActuator_2.set(position);

  }

  public void move_top_conveyor(final double speed) {
    topConveyorTalon.set(speed);
  }

  private double clipLinearActuatorPositionCommand(double position) {
    // Make sure command does not exceed the hardware limit
    if (position > Constants.UPPER_SERVO_POS_LIMIT) {
      position = Constants.UPPER_SERVO_POS_LIMIT;
    }

    if (position < Constants.LOWER_SERVO_POS_LIMIT) {
      position = Constants.LOWER_SERVO_POS_LIMIT;
    }
    return position;
  }

  public boolean servo_at_position(final double endPosition) {
    final double actuator_1_position = angleActuator_1.getPosition();
    final double actuator_2_position = angleActuator_2.getPosition();

    final double actuator_1_position_delta = Math.abs(actuator_1_position - endPosition);
    final double actuator_2_position_delta = Math.abs(actuator_2_position - endPosition);

    return ((actuator_1_position_delta + actuator_2_position_delta) < 0.04);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter motor speed", getVelocity_rpm());
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.utils.HermitClampedCubicPath;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.geometry.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Ports on the driver station where the contollers are connected
    public static final int DRIVER_CONTROLLER = 0;
    public static final int MANIPULATOR_CONTROLLER = 1;

    // Places where buttons are on a given controller, create these as JoystickButtons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int D_PAD_UP = 5;
    public static final int D_PAD_RIGHT = 6;
    public static final int D_PAD_DOWN = 7;
    public static final int D_PAD_LEFT = 8;
    public static final int L2 = 9;
    public static final int R2 = 10;
    public static final int L1 = 11;
    public static final int R1 = 12;

    // Definition of joystick axes variables for a controller joystick
    public static final int left_x_axis = 0;
    public static final int left_y_axis = 1;

    // Drivetrain controller CAN IDs
  // Used for Chassis Bot
//     public static final int RIGHT_MOTOR_1 = 1;
//     public static final int RIGHT_MOTOR_2 = 3;
//     public static final int LEFT_MOTOR_1 = 2;
//     public static final int LEFT_MOTOR_2 = 4;
    public static final int DRVTRN_SOL_FWD_CHN = 0;
    public static final int DRVTRN_SOL_RVS_CHN = 1;

    public static final int RIGHT_MOTOR_1 = 11;
    public static final int RIGHT_MOTOR_2 = 8;
    public static final int LEFT_MOTOR_1 = 5;
    public static final int LEFT_MOTOR_2 = 12;

    public static final int LEFT_ENCODER_channel1A = 0;
    public static final int LEFT_ENCODER_channel1B = 1;
    public static final int RIGHT_ENCODER_channel1A = 2;
    public static final int RIGHT_ENCODER_channel1B = 3;


    // This does not exist yet!
    // public static final int ROTATE_TURRET_MOTOR = 10; // Does not exist
    public static final int TOP_CONVEYOR_MOTOR = 1;
    public static final int LOWER_CONVEYOR_MOTOR = 4;
    public static final int INTAKE_MOTOR = 7;
   
    // Shooter controller CAN IDs
    public static final int SPIN_SHOOTER_MOTOR_1 = 2;
    public static final int SPIN_SHOOTER_MOTOR_2 = 3;

    // PWM channels for servo controller linear actuators
    public static final int LINEAR_ACTUATOR_1 = 1;
    public static final int LINEAR_ACTUATOR_2 = 2;

    // Position limits for servo linear actuators
    public static final double UPPER_SERVO_POS_LIMIT = 0.87; // All the way out
    public static final double LOWER_SERVO_POS_LIMIT = 0.17; // All the way in

    // Elevation system controller CAN IDs
    public static final int ELEVATOR_TALON_FX = 40;
    public static final double ELEVATOR_SPEED = 0.6;
    
    // Controller constants
    public static final double visionPID_Clip = 0.4;
    public static final double visionKp = 0.4;
    public static final double visionKi = 0.5;
    public static final double visionKd = 0.25;
    public static final double alignment_x_tolerance = 0.05;

    // Define Path 1
    private static final Pose2d startPose_1 = new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0));
    private static final Pose2d endPose_1 = new Pose2d(new Translation2d(2.0,2.0), Rotation2d.fromDegrees(-45));
    private static final ArrayList<Translation2d> interiorWaypoints_1 = new ArrayList<Translation2d>() {
      {
        add(new Translation2d(0.5,1.0));
        add(new Translation2d(1.5,1.0));
      }
    };
    private static final HermitClampedCubicPath PATH_1 = new HermitClampedCubicPath(startPose_1,interiorWaypoints_1,endPose_1,3.5,3.5);

    // Define Path 2
    private static final Pose2d startPose_2 = new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0));
    private static final Pose2d endPose_2 = new Pose2d(new Translation2d(2.0,-2.0), Rotation2d.fromDegrees(-45));
    private static final ArrayList<Translation2d> interiorWaypoints_2 = new ArrayList<Translation2d>() {
      {
        add(new Translation2d(0.5,-0.5));
        add(new Translation2d(1.5,-1.0));
      }
    };
    private static final HermitClampedCubicPath PATH_2 = new HermitClampedCubicPath(startPose_2,interiorWaypoints_2,endPose_2,3.5,3.5);

    // Create array list of paths
    public static final ArrayList<HermitClampedCubicPath> PATH_LIST = new ArrayList<HermitClampedCubicPath>() {
      {
        add(PATH_1);
        add(PATH_2);
      }
    };

    // Trajectory following parameters
    public static final double TrackWidth = 23.75; // inches
    public static final double WheelRadius = 8.00; // inches
    public static final double TBD = 1.0; // inches
    public static final double EncoderResolution = 4096; // number of pulses per revolution
    public static final double GR1 = 9.0; // 1st gear ratio
    public static final double GR2 = 9.0; // 2nd gear ratio
    public static final double RobotCharacterization_kS = 0.3; // static gain
    public static final double RobotCharacterization_kV = 1.96; // velocity gain
    public static final double RobotCharacterization_kA = 0.06; // acceleration gain
    public static final double Traj_Following_Feedback_P_Gain = 2.95; // P gain for PID

    // Calibration of shooter
    public static final double[] distances = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
    }; // feet, need to change to inches

    // Speed (% output) of the shooter motors
    public static final double[] speeds = {
      0.66, 0.68, 0.70, 0.75, 0.75, 0.75, 0.80, 0.90, 0.90, 0.90, 0.90, 0.90
    };

    // Position (PWM command) of the linear actuators
    public static final double[] position = {
      0.85, 0.75, 0.75, 0.65, 0.65, 0.65, 0.55, 0.55, 0.55, 0.45, 0.45, 0.45, 0.45
    };

    // Parameters for vision subsystem
    public static final float LIMELIGHT_VERTICAL_ANGLE = (float)0.0;    // Limelight vertical angle degrees
    public static final float LIMELIGHT_HEIGHT = (float)0.0;            // Limelight height from ground
    public static final float TARGET_HEIGHT = (float)0.0;               // Target height from ground

    // Slew rate limiters
    public static final double SLEW_RATE_LIMIT_ROTATE = 0.5;
    public static final double SLEW_RATE_LIMIT_ACCEL = 0.5;

    Constants() {
    }
}

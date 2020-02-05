/*----------------------------------------------------------------------------*/
/* 1/22/2020 v1                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Limelight imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;

import java.lang.Math;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  
  // Limelight object
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");

  private final float A1 = (float)0.0;     // Measure, move to other class?
  private final float H1 = (float)5.01;     // Measure, move to other class?
  private final float H2 = (float)6.1;     // Measure, move to other class?

  private double[] ffGains = {
      0.0008171388625648901,
      0.0025796090816614394,
      0.004245625441810102,
      0.0028920364306526743,
      -0.004485549864848663,
      -0.017206206747234075,
      -0.027692599432802778,
      -0.022583572720391073,
      0.01028905933557547,
      0.07228314186855418,
      0.14849473849283668,
      0.21195572576869964,
      0.23668096456728935,
      0.21195572576869964,
      0.14849473849283668,
      0.07228314186855418,
      0.01028905933557547,
      -0.022583572720391073,
      -0.027692599432802778,
      -0.017206206747234075,
      -0.004485549864848663,
      0.0028920364306526743,
      0.004245625441810102,
      0.0025796090816614394,
      0.0008171388625648901
  };

  private LinearFilter filter_d = new LinearFilter(ffGains, null);
  private LinearFilter filter_ha = new LinearFilter(ffGains, null);
  private LinearFilter filter_va = new LinearFilter(ffGains, null);

  private double filtered_distance;
  private double distance;
  private double filtered_h_angle;
  private double h_angle;
  private double filtered_v_angle;
  private double v_angle;
  
  public VisionSubsystem() {      
  }

  // Limelight x
  public double getHorizontalAngle() {
    return(h_angle);
  }

  // Limelight y
  public double getVerticalAngle() {
    return(v_angle);
  }

  // Limelight area
  public double getTargetArea() {
    double a = ta.getDouble(0.0);
    return(a);
  }

  // Limelight target detected flag
  public double getCaptureStatus() {
    double v = tv.getDouble(0.0);
    return(v);
  }

  // Calculate distance to target
  private double calculateDistance() {
    double dist = -1;
    
    if (getCaptureStatus() == 1.0) {
        dist = (H2-H1)/Math.tan(A1+getVerticalAngle());
    }

    return(dist);
  }

  // Return distance
  public double getDistance() {
    return(distance);
  }

  // Return filtered horizontal angle
  public double getFilteredHorizontalAngle() {
    return(filtered_h_angle);
  }

  // Return filtered vertical angle
  public double getFilteredVerticalAngle() {
    return(filtered_v_angle);
  }

  // Return filtered distance
  public double getFilteredDistance() {
    return(filtered_distance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = calculateDistance();
    filtered_distance = filter_d.calculate(distance);
    h_angle = tx.getDouble(0.0);
    filtered_h_angle = filter_ha.calculate(h_angle);
    v_angle = ty.getDouble(0.0);
    filtered_v_angle = filter_va.calculate(v_angle);

    SmartDashboard.putNumber("LimelightHorizontalAngle", getHorizontalAngle());
    SmartDashboard.putNumber("LimelightVerticalAngle", getVerticalAngle());
    SmartDashboard.putNumber("LimelightArea", getTargetArea());
    SmartDashboard.putNumber("LimelightCaptureStatus", getCaptureStatus());
    SmartDashboard.putNumber("LimelightDistance", distance);
    SmartDashboard.putNumber("LimelightFilteredDistance", filtered_distance);
    SmartDashboard.putNumber("LimelightFilteredHorizontalAngle", filtered_h_angle);
    SmartDashboard.putNumber("LimelightFilteredVerticalAngle", filtered_v_angle);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  double x;
  double y;
  double area;
  double target;

  public Limelight() {}

  @Override
  public void periodic() {
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = tv.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("X shot confidence", 1 / (Math.abs(x) - Constants.X_CROSS_OFFSET));
    SmartDashboard.putNumber("Y shot confidence", 1 / (Math.abs(y) - Constants.Y_CROSS_OFFSET));
  }
  /**
   * 
   * @param var the name of the variable that you want to get
   * x - the x angle of the target (tx)
   * y - the y angle of the target (ty)
   * a - the area of the target (ta)
   * @return a double
   */
  public boolean hasTarget() {
    return target == 1 ? true : false;
  }

  public double getTarget(String var) {
    if (var == "x"){
      return x;
    } else if (var == "y") {
      return y;
    } else if (var == "a") {
      return area;
    } else {
      return 0;
    }
  }

  public double highGoalDistance () {
    return (Constants.HIGHGOAL_HEIGHTM - Constants.LIME_HEIGHTM) / (Math.tan(Math.toRadians(y)) - Constants.LIME_ANGLE);
  }
}

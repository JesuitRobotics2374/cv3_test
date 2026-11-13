// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.constants;

public final class DriverConstants {
  public static final int DRIVER_CONTROLLER_PORT = 0;

  /**
   * How far the left stick must be pushed before its direction is treated as a real command. Near
   * the center the reported angle is mostly noise, so what matters is a magnitude threshold rather
   * than a per-axis deadband -- we care which way the stick points, not how far.
   */
  public static final double STICK_ANGLE_DEADBAND = 0.5;


}

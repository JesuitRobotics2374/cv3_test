// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;
import first.robot.constants.DriverConstants;
import first.robot.mechanisms.Arm;

public class Robot extends OpModeRobot {
  public final Arm arm = new Arm();
  final CommandGamepad driverController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  public Robot() {
    // Whenever nothing else claims the arm hold arm position.
    arm.setDefaultCommand(arm.hold());
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}

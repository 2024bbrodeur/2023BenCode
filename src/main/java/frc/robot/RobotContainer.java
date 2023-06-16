// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ModuleTestCommand;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
  
  private final CommandJoystick leftJoystick = new CommandJoystick(Constants.Joysticks.LEFT_JOYSTICK_PORT);
  private final CommandJoystick rightJoystick = new CommandJoystick(Constants.Joysticks.RIGHT_JOYSTICK_PORT);
  private final CommandXboxController controller = new CommandXboxController(Constants.Joysticks.CONTROLLER_PORT);
  
  
  
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData(new ModuleTestCommand(new Module(new ModuleIOTalonFX(0), 0)));
  }

  private void configureBindings() {}
}

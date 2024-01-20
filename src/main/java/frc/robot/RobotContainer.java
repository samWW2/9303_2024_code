// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.PS4Controller;





public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS4Controller  m_XboxController = new PS4Controller (0);
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS4Controller.Axis.kRightX.value;
  
  private final Trigger robotCentric = new JoystickButton(m_XboxController,PS4Controller.Button.kSquare.value);
  private final Trigger xButton = new JoystickButton(m_XboxController, PS4Controller.Button.kCross.value);
  private final Trigger oButton = new JoystickButton(m_XboxController, PS4Controller.Button.kCircle.value);



  /*private final int translationAxis = Joystick.AxisType.kY.value; //left flight stick
  private final int strafeAxis = Joystick.AxisType.kX.value; //left flight stick
  private final int rotationAxis = Joystick.AxisType.kX.value; //right flight stick*/

  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();



  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_XboxController.getRawAxis(translationAxis),
          () -> -m_XboxController.getRawAxis(strafeAxis),
          () -> -m_XboxController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));

    configureBindings();
  }

 
  private void configureBindings() {
    oButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    xButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX()));
   
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}

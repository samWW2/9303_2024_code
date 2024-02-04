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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS5Controller;





public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  private final PS5Controller m_PS5Controller = new PS5Controller(0);


  /* Drive Controls */
  private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS5Controller.Axis.kRightX.value;
  
  private final Trigger robotCentric = new JoystickButton(m_PS5Controller,PS5Controller.Button.kSquare.value);
  private final Trigger xButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCross.value);
  private final Trigger oButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCircle.value);
  private final Trigger touchpad = new JoystickButton(m_PS5Controller, PS5Controller.Button.kTouchpad.value);


  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();



  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_PS5Controller.getRawAxis(translationAxis),
          () -> -m_PS5Controller.getRawAxis(strafeAxis),
          () -> -m_PS5Controller.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));

    configureBindings();
  }

 
  private void configureBindings() {
    oButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    xButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX()));
    // touchpad.onTrue(new InstantCommand(() -> m_SwerveSubsystem.resetAll()));
   
  }

 
  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
    return AutoBuilder.followPath(path);
  }
}

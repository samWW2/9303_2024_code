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

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public class RobotContainer {

  private Command visionAuto(){
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 5.5, Rotation2d.fromDegrees(0)));

        PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));

     path.preventFlipping =true;
     m_SwerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path);
  }

  private Command Auto1(){
    m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 1"));
    return AutoBuilder.buildAuto("Auto 1");
  }
   private Command Auto2(){
    m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 2"));
    return AutoBuilder.buildAuto("Auto 2");
  }
   private Command Auto3(){
    m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 3"));
    return AutoBuilder.buildAuto("Auto 3");
  }
   private Command Auto4(){
    m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 4"));
    return AutoBuilder.buildAuto("Auto 4");
  }
   private Command Auto5(){
    m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 5"));
    return AutoBuilder.buildAuto("Auto 5");
  }
  public Command NullAuto(){
    return null;
  }
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private final PS5Controller m_PS5Controller = new PS5Controller(0);

  /* Drive Controls */
  private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS5Controller.Axis.kRightX.value;
  
  private final Trigger robotCentric = new JoystickButton(m_PS5Controller,PS5Controller.Button.kSquare.value);
  private final Trigger xButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCross.value);
  private final Trigger oButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCircle.value);


  /* Subsystems */
  public static final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

  
  
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
    chooser.setDefaultOption("no auto", NullAuto());
    chooser.addOption("Auto 1", Auto1());
    chooser.addOption("Auto 2", Auto2());
    chooser.addOption("Auto 3", Auto3());
    chooser.addOption("Auto 4", Auto4());
    chooser.addOption("Auto 5", Auto5());
    chooser.addOption("vision auto", visionAuto());
    SmartDashboard.putData(chooser);
   
  }

 
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Back Left 45");
    // m_SwerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(path);
    return chooser.getSelected();
  }
}

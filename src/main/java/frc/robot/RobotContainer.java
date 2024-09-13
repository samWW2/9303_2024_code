// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetIntakeWithDelay;
// import frc.robot.commands.ClimberUp;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.shootWithTime;
// import frc.robot.subsystems.Climber;6
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public class RobotContainer {

 

 
  // private SendableChooser<Command> chooser;
  private final PS5Controller m_PS5Controller = new PS5Controller(0);
  private final CANSparkMax motor32 = new CANSparkMax(32, MotorType.kBrushless);
  // private final GenericHID joystick = new GenericHID(1);

  /* Drive Controls */
  private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS5Controller.Axis.kRightX.value;
  
  private final Trigger robotCentric = new JoystickButton(m_PS5Controller,PS5Controller.Button.kCross.value);
  private final Trigger oButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kCircle.value);
  private final Trigger sqrButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kSquare.value);
  private final Trigger triButton = new JoystickButton(m_PS5Controller, PS5Controller.Button.kTriangle.value);


  // private final JoystickButton climberUpButton = new JoystickButton(joystick, 1);
  // private final JoystickButton climberDownButton = new JoystickButton(joystick, 2);
  // private final Trigger intakeButtonIn = new JoystickButton(joystick,7); 
  // private final Trigger intakeButtonOut = new JoystickButton(joystick,1); 
  // private final Trigger shootAmpButton = new JoystickButton(joystick,6);
  // private final Trigger feederButton = new JoystickButton(joystick,5);
  // private final Trigger shootSpekerButton = new JoystickButton(joystick,8);
  // private final Trigger climberDownButton = new JoystickButton(joystick,1);
  // private final Trigger climberUpButton = new JoystickButton(joystick,2); 




private final SwerveSubsystem m_SwerveSubsystem;
// private final Intake m_intake;
// private  final Climber climber;
// private final ShooterSubsystem shootSub;
// private final LimeLight vision;

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    motor32.setIdleMode(IdleMode.kBrake);
       /* Subsystems */
   m_SwerveSubsystem = new SwerveSubsystem();
  //  m_intake = new Intake();
  //  climber = new Climber();
  // vision= new LimeLight();
  //  shootSub= new ShooterSubsystem();

  //  NamedCommands.registerCommand("shootSpeakerAuto", shootSpeakerAuto());
  //  chooser=new SendableChooser<>();
   
    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_PS5Controller.getRawAxis(translationAxis),
          () -> -m_PS5Controller.getRawAxis(strafeAxis),
          () -> -m_PS5Controller.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));

    configureBindings();
  }
   // private Command visionAuto(){
  //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  //       m_SwerveSubsystem.getPose(),
  //       vision.getTagPose());

  //       PathPlannerPath path = new PathPlannerPath(
  //       bezierPoints,
  //       new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
  //       new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

  //    path.preventFlipping =true;
  //    m_SwerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
  //   return AutoBuilder.followPath(path);
  // }
  
  // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
  // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
  // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
  // private Command testAuto(){
  //      PathPlannerPath path = PathPlannerPath.fromPathFile("Work");
  //      path.preventFlipping =true;
  //      return AutoBuilder.followPath(path);
  // }
  // private Command shootAuto(){
  //   return new SequentialCommandGroup(
  //     new RunCommand(() -> shootSub.setshootspeedCommand(-0.35, 0.6).until(shootSub.isatSetpoint()),
  //     new ParallelCommandGroup(
  //     new RunCommand(() -> shootSub.setshootspeedCommand(-0.35, 0.6).withTimeout(3)),
  //     new RunCommand(() -> m_intake.setintakemotorspeed(0.5).withTimeout(3))
  //       )
  //      )
  //     );
  // }
  // private Command shootAndPickUpM(){
  //   m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("ShootAndPickUpM"));
  //   return AutoBuilder.buildAuto("ShootAndPickUpM");
  // }
  // private Command shootAuto(){
  //   m_SwerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Shoot"));
  //   return AutoBuilder.buildAuto("Shoot");
  // }
  // private Command shootSpeakerAuto(){
  //   return new ParallelCommandGroup(
  //     new shootWithTime(shootSub, -0.35, 0.65, 4),
  //     new SetIntakeWithDelay(m_intake, 0.5, 3, 4.5)
  //     );
  // }
  public Command nullAuto(){
    return null;
  }

 
  private void configureBindings() {
    oButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    sqrButton.onTrue(new InstantCommand(() -> motor32.set(0.1)));
    sqrButton.onFalse(new InstantCommand(() -> motor32.set(0)));
    triButton.onTrue(new InstantCommand(() -> motor32.set(-0.1)));
    triButton.onFalse(new InstantCommand(() -> motor32.set(0)));

    // intakeButtonIn.onTrue(new InstantCommand(() -> m_intake.setintakemotors(0.35)));
    // intakeButtonIn.onFalse(new InstantCommand(() -> m_intake.setintakemotors(0)));
    // intakeButtonOut.onTrue(new InstantCommand(() -> m_intake.setintakemotors(-0.5)));
    // intakeButtonOut.onFalse(new InstantCommand(() -> m_intake.setintakemotors(0)));
    // shootAmpButton.onTrue(new InstantCommand(() -> shootSub.setshootmotorPercent(-0.42, 0.42)));
    // shootAmpButton.onFalse(new InstantCommand(() -> shootSub.setshootmotorPercent(0, 0)));
    // feederButton.onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_intake.setintakemotors(-0.2)),new InstantCommand(() -> shootSub.setshootmotorPercent(0.5, -0.5))));
    // feederButton.onFalse(new ParallelCommandGroup(new InstantCommand(() -> m_intake.setintakemotors(0)),new InstantCommand(() -> shootSub.setshootmotorPercent(0, 0))));
    // shootSpekerButton.onTrue(new InstantCommand(() -> shootSub.setshootmotorPercent(-0.35, 1)));
    // shootSpekerButton.onFalse(new InstantCommand(() -> shootSub.setshootmotorPercent(0, 0)));
    // climberDownButton.onTrue(new InstantCommand(() -> climber.setMotors(-0.5)));
    // climberDownButton.onFalse(new InstantCommand(() -> climber.setMotors(0)));
    // climberUpButton.onTrue(new InstantCommand(() -> climber.setMotors(0.5)));
    // climberUpButton.onFalse(new InstantCommand(() -> climber.setMotors(0)));


    oButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    // climberUpButton.onTrue(new ClimberUp(climber, 0.5));
    // climberUpButton.onFalse(new InstantCommand(() -> climber.stop()));
    // climberDownButton.onTrue(new ClimberUp(climber, -0.5));
    // climberDownButton.onFalse(new InstantCommand(() -> climber.stop()));
    // chooser.setDefaultOption("no auto", nullAuto());
    // chooser.addOption("shootAndPickUpM", shootAndPickUpM());
    // chooser.addOption("shoot", shootAuto());
    // chooser.addOption("new shoot", shootSpeakerAuto());
    // // chooser.addOption("vision auto", visionAuto());
    // chooser.addOption("test", testAuto());
    // SmartDashboard.putData(chooser);
   
  }

 
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Back Left 45");
    // m_SwerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(path);
    return null;
  }
}

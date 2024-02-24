// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Climber;

// public class ClimberUp extends Command {
//   private final Climber climber;
//   private final double speed;
//   public ClimberUp(Climber climber,double speed) {
//     this.climber = climber;
//     this.speed=speed;
//     addRequirements(climber);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     climber.setMotors(speed);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     climber.stop();
//   }

//   @Override
//   public boolean isFinished() {
//     if(climber.isDetected()){
//       return true;
//     }
//     return false;
//   }
// }

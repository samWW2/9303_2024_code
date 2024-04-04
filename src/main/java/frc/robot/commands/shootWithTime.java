// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shootWithTime extends Command {
  Timer timer;
  ShooterSubsystem shoot;
  double topSpeed;
  double buttomSpeed;
  double time;
  public shootWithTime(ShooterSubsystem shoot, double topSpeed, double buttomSpeed, double time ) {
    this.shoot = shoot;
    this.topSpeed = topSpeed;
    this.buttomSpeed = buttomSpeed;
    this.time = time;
    timer = new Timer();
    addRequirements(shoot);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    shoot.setshootmotorPercent(topSpeed, buttomSpeed);
    SmartDashboard.putNumber("time", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()> time){
      return true;
    }
    return false;
  }
}

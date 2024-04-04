// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeWithDelay extends Command {
  Timer timer;
  Intake intake;
  double speed;
  double delay;
  double timeout;
  public SetIntakeWithDelay(Intake intake, double speed, double delay, double timeout) {
    this.intake = intake;
    this.speed = speed;
    this.delay = delay;
    timer = new Timer();
    addRequirements(intake);
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if(timer.get() > delay)
    {
      intake.setintakemotors(0.5);
    }
   
    SmartDashboard.putNumber("time", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setintakemotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()> timeout){
      return true;
    }
    return false;
  }
}
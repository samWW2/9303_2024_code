// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class Intake extends SubsystemBase {
  private final CANSparkMax intakemotor1;
  private final CANSparkMax intakemotor2;

  

  public Intake() {
     this.intakemotor1= new CANSparkMax(45, MotorType.kBrushless);
     this.intakemotor2= new CANSparkMax(50, MotorType.kBrushless);
     intakemotor1.setInverted(true);
  }

  @Override
  public void periodic() {
  }
  public void setintakemotors(double motorspeed){
    intakemotor1.set(motorspeed);
    intakemotor2.set(motorspeed);
  }

 public Command setintakemotorspeed(double intakemotorspeed){
  return new RunCommand(()-> setintakemotors(intakemotorspeed));
 }


  public void stopMotors() {
    intakemotor1.stopMotor();
    intakemotor2.stopMotor();
  }

}

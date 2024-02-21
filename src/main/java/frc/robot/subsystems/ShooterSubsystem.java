// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topshootmotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax bottomshootmotor = new CANSparkMax(2, MotorType.kBrushless);
  private final PIDController shooterSpeedController = new PIDController(0, 0, 0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topshootmotor.setInverted(true);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("topshootmotorspeed",gettopVelocity());
    SmartDashboard.putNumber("bottomshootmotorspeed",getbottomVelocity());
  }
  public void setshootmotor(double topshootingSpeed,double bottomshootingSpeed){
    topshootmotor.set(shooterSpeedController.calculate(gettopVelocity(),topshootingSpeed));
    bottomshootmotor.set(shooterSpeedController.calculate(getbottomVelocity(),bottomshootingSpeed));

  }
  public Command setshootspeedCommand(double topshootspeed,double bottomshootspeed){
    return new RunCommand(()-> setshootmotor(topshootspeed,bottomshootspeed));
  }

  public boolean isatSetpoint(){
    return shooterSpeedController.atSetpoint();
  }
  public void stopmotors(){
    topshootmotor.stopMotor();
  }
  private double gettopVelocity(){
    return topshootmotor.getEncoder().getVelocity();
  }
  private double getbottomVelocity(){
    return bottomshootmotor.getEncoder().getVelocity();
  }
}

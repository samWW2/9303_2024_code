

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

  public ShooterSubsystem() {
    topshootmotor.setInverted(true);
    bottomshootmotor.follow(topshootmotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shootmotorspeed",getVelocity());
  }

  public Command setspeedCommand(double shootingSpeed){
    return new RunCommand(()-> topshootmotor.set(shooterSpeedController.calculate(getVelocity(),shootingSpeed)));
  }
  public boolean isatSetpoint(){
    return shooterSpeedController.atSetpoint();
  }
  public void stopmotors(){
    topshootmotor.stopMotor();
  }
  private double getVelocity(){
    return topshootmotor.getEncoder().getVelocity();
  }
}

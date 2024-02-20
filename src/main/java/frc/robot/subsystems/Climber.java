

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberLeft;
  private final CANSparkMax climberRight;
  private final RelativeEncoder leftEnc;
  private final RelativeEncoder rightEnc;
  private DigitalInput hallEffectSensor;

  
  public Climber() {
    climberLeft = new CANSparkMax(Constants.ClimberConstants.leftClimberID, MotorType.kBrushless);
    climberLeft.setIdleMode(Constants.ClimberConstants.mode);
    climberRight = new CANSparkMax(Constants.ClimberConstants.rightClimberID, MotorType.kBrushless);
    climberRight.setIdleMode(Constants.ClimberConstants.mode);
    leftEnc = climberLeft.getEncoder();
    leftEnc.setPositionConversionFactor(Constants.ClimberConstants.encoderPositionConversionFactor);
    rightEnc = climberRight.getEncoder();
    rightEnc.setPositionConversionFactor(Constants.ClimberConstants.encoderPositionConversionFactor);

    hallEffectSensor = new DigitalInput(Constants.ClimberConstants.digInputHESport);
  }
  public void setMotors(double speed){
    climberLeft.set(speed);
    climberRight.set(speed);
  }
  public void stop(){
    climberLeft.stopMotor();
    climberRight.stopMotor();
  }
  public double getLeftPositionInCm(){
    return leftEnc.getPosition();
  }
  public double getRightPositionCm(){
    return rightEnc.getPosition();
  }
  public boolean isDetected(){
    boolean isMagneticFieldDetected = hallEffectSensor.get();
     if (isMagneticFieldDetected) { System.out.println("Magnetic field detected!");}
     else { System.out.println("No magnetic field detected.");}
     return isMagneticFieldDetected;
  }

  @Override
  public void periodic() {
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    gyro = new AHRS(SPI.Port.kMXP);
    zeroGyro();
    
    //Creates all four swerve modules into a swerve drive
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod3.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod2.constants)
    };
    
    //creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);

     // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelitive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.01, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates =
      Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  -translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(-translation.getX(), translation.getY(), rotation));
  //sets to top speed if above top speed
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

  //set states for all 4 modules
  for (SwerveModule mod : mSwerveMods) {
    mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  }
    SmartDashboard.putNumber("set point ", swerveModuleStates[0].angle.getDegrees());
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
  }


  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelitive(ChassisSpeeds targetSpeed)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeed,
        new Translation2d());
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);// MIGHT NEED TO CHANGE
    }
  }
  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
      // front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      // front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      // back left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
      // back right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}


  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertNavx)
        ? Rotation2d.fromDegrees((360 - gyro.getYaw()))
        : Rotation2d.fromDegrees((gyro.getYaw() + Constants.SwerveConstants.degreesOffSet));
  }

 

  public boolean AutoBalance(){
    double roll_error = gyro.getPitch();//the angle of the robot
    double balance_kp = -.005;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .1){position_adjust = .1;}
      if (position_adjust < -.1){position_adjust = -.1;}
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      
      return false;
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      drive(new Translation2d(0, 0), 0.0, true, false);
      return true;}
    
  }
  public double getAngle(SwerveModule mod){
    if(mod.getState().angle.getDegrees() > 360){
      return mod.getState().angle.getDegrees() -360;
  }
  else if(mod.getState().angle.getDegrees() < 0){
       return mod.getState().angle.getDegrees() + 360;
  }
    return mod.getState().angle.getDegrees();
  }



  @Override
  public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("gyro yaw",  getYaw().getDegrees());
    SmartDashboard.putNumber("robot pose x", getPose().getX());
    SmartDashboard.putNumber("robot pose y", getPose().getY());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated Angle", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }
}


}

// package frc.robot.subsystems;

// import java.io.IOException;
// import java.util.Arrays;
// import java.util.HashMap;
// import java.util.List;


// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.helpers.LimelightHelpers;

// public class LimeLight extends SubsystemBase {
   
//   //create NetworkTable objects
//   NetworkTableInstance nInstance;
//   NetworkTable table; 
  
//   //limelight values
//   private NetworkTableEntry ta; 
//   private NetworkTableEntry tv; 
//   private NetworkTableEntry ty;
//   private NetworkTableEntry tx; 
//   private NetworkTableEntry tid; 
//   private double[] tagpose, botpose; 
//   private int pipeline; 

//   //trajectory fields
//   private Trajectory trajectory;

//   HashMap<Double, Pose2d> poses; 

//   public LimeLight() 
//   {
//       System.out.println("Limelight object initialized");

//       nInstance = NetworkTableInstance.getDefault();
//       table = nInstance.getTable("limelight");
//       ta = table.getEntry("ta");
//       tv = table.getEntry("tv");
//       tx = table.getEntry("tx");
//       ty = table.getEntry("ty");
//       tid = table.getEntry("tid");
//       tagpose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); 
//       botpose = table.getEntry("robotpose_targetspace").getDoubleArray(new double[6]); 
//       poses = new HashMap<Double, Pose2d>();
  
//   }

//   public Pose2d getTagPose() { 
//       return new Pose2d(getLLData().getY()+2.5, getLLData().getX(), Rotation2d.fromDegrees(0));
//   }
//   public Pose2d getLLData(){
//     return LimelightHelpers.getBotPose2d("limelight");
//   }
//   public void printTargetPoses() {
//       System.out.println(Arrays.asList(poses));
//   }

//   public double getTa() {
//       return ta.getDouble(0); 
//   }

//   public double getTv() {
//       return tv.getDouble(0); 
//   }

//   public double getTx() {
//       return tx.getDouble(0); 
//   }

//   public double getTy() {
//       return ty.getDouble(0); 
//   }

//   public double getTid() {
//       return tid.getDouble(0); 
//   }

//   public String getPipeline() {
//       return "Currently using pipeline " + pipeline; 
//   }

//   public void setPipeline(int pipeline) {
//       this.pipeline = pipeline;
//       table.getEntry("pipeline").setNumber(pipeline);
//       System.out.println("Pipeline changed to " + pipeline); 
//   }
 
//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("x value", getTagPose().getX());
//     SmartDashboard.putNumber("y value", getTagPose().getY());
//       super.periodic();
//   }
//   public void mapOriginPairs() {
//       //initialize dictionary 
//       if (this.getTv() == 1) {
//           poses.put(this.getTid(), this.getTagPose());
//           System.out.println("Successfully mapped");
//       }

//       else {
//           System.out.println("No mapping to be done");
//       }

//   }

//   public Trajectory generateTargetTrajectory(TrajectoryConfig config) {
//       System.out.println("Trajectory generated successfully"); 
//       //set tag pose as the current origin 
//       //origin = this.getTagPose();
//       if (this.getTv() == 1) {
//           RobotContainer.m_SwerveSubsystem.resetOdometry(RobotContainer.m_SwerveSubsystem.getPose()); //sets origin to tag pose 
  
//           trajectory = TrajectoryGenerator.generateTrajectory(
//               RobotContainer.m_SwerveSubsystem.getPose(),
//               List.of(),
//               this.getTagPose(),
//               config);

//       }
      
//       return trajectory; 
  
//   }


// }

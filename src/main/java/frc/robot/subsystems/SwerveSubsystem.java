// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collective

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class SwerveSubsystem extends SubsystemBase
{ 
  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED,
      DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_PORT,
      DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
      DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED,
      "Front Left");  
      
  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_TURNING_ENCODING_REVERSED,
      DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_PORT,
      DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
      DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED,
      "Front Right");
  
  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.BACK_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.BACK_LEFT_TURNING_MOTOR_ID,
      DriveConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
      DriveConstants.BACK_LEFT_TURNING_ENCODER_REVERSED,
      DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_PORT,
      DriveConstants.BACK_LEFT_ASOLUTE_ENCODER_OFFSET,
      DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_REVERSED,
      "Back Left");
  
  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.BACK_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
      DriveConstants.BACK_RIGHT_TURNING_ENCODER_REVERSED,
      DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_PORT,
      DriveConstants.BACK_LEFT_ASOLUTE_ENCODER_OFFSET,
      DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED,
      "Back Right");
  
  // Create the navX using roboRIO expansion port
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Create odometer for swerve drive
  private final SwerveDriveOdometry odometer;

  // Create Field2D for keeping track of Robot Position
  private final Field2d field = new Field2d();
  
  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions()
  {
    return (new SwerveModulePosition[]
    {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    });
  }
  
  // Swerve subsystem constructor
  public SwerveSubsystem()
  {
    // Reset robot encoders on startup
    resetAllEncoders();

    // Add Gyro and Robot Position to Shuffleboard
    ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Robot Position", field);
    ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Gyro", gyro);
    
    // Zero navX heading on new thread when robot starts
    new Thread(() ->
    {
      try
      {
        Thread.sleep(1000);
        zeroHeading();
      }
      catch (Exception e)
      {
      }
    }).start();
    
    // Set robot odometry object to current robot heading and swerve module
    // positions
    odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS,
        getOdometryAngle(), getModulePositions());

    // Set robot position on Field
    field.setRobotPose(getPose());
  }
  
  // Reset gyro heading
  public void zeroHeading()
  {
    gyro.reset();
  }
  
  // Reset gyro yaw
  public void resetYaw()
  {
    gyro.zeroYaw();
  }
  
  // Return gyro heading, make sure to read navx docs on this
  public double getHeading()
  {
    return gyro.getAngle();
  }
  
  // Return heading in Rotation2d format
  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading());
  }
  
  // Stop all module movement
  public void stopModules()
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
  
  // Move the swerve modules to the desired SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.DRIVE_MAX_LINEAR_SPEED);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
  
  // Return robot position caculated by odometer
  public Pose2d getPose()
  {
    return odometer.getPoseMeters();
  }
  
  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose)
  {
    odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }
  
  // Reset odometer to new Pose2d location but with roation
  public void resetOdometry(Pose2d pose, Rotation2d rot)
  {
    odometer.resetPosition(rot, getModulePositions(), pose);
  }
  
  // Return an angle from -180 to 180 for robot odometry
  public Rotation2d getOdometryAngle()
  {
    return (Rotation2d.fromDegrees(gyro.getYaw() * -1));
  }
  
  // Returns an angle from 0 to 360 that is continuous, meaning it loops
  public double getRobotDegrees()
  {
    double rawValue = 360 - (gyro.getAngle() % 360.0);

    // Check if value needs to wrap around
    if (rawValue < 0.0)
    {
      return (rawValue + 360.0);
    }
    else
    {
      return (rawValue);
    }
  }
  
  // Reset all swerve module encoders
  public void resetAllEncoders()
  {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }
  
  @Override
  public void periodic()
  {
    // Update odometer for it to caculate robot position
    odometer.update(getOdometryAngle(), getModulePositions());

    // Update robot position on Field
    field.setRobotPose(getPose());
    
    // Update smartdashboard data for each swerve module object
    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();
  }
}
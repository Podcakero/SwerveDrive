// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collective

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShuffleboardConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveJoystickCommand extends Command
{
  // Create variables
  private final SwerveSubsystem SWERVE_SUBSYSTEM;
  private final SlewRateLimiter X_LIMITER, Y_LIMITER;
  private final PIDController ROTATION_CONTROLLER;

  private double targetHeading;

  // Shuffleboard Components
  private final GenericEntry TARGET_HEADING_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Target Heading", targetHeading).getEntry();

  private final GenericEntry X_SPEED_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("X Speed", 0.0).getEntry();
  private final GenericEntry Y_SPEED_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Y Speed", 0.0).getEntry();

  private final GenericEntry TURNING_SPEED_BEFORE_PID_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Turning Speed Before PID", 0.0).getEntry();
  private final GenericEntry TURNING_SPEED_AFTER_PID_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Turning Speed After PID", 0.0).getEntry();
  
  // Command constructor
  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem)
  {
    // Assign values passed from constructor
    this.SWERVE_SUBSYSTEM = swerveSubsystem;
    
    // Slew rate limiter
    // This smooths out the behavior of converting the Joystick values to Swerve Movements
    this.X_LIMITER = new SlewRateLimiter(DriveConstants.DRIVE_MAX_LINEAR_ACCELERATION);
    this.Y_LIMITER = new SlewRateLimiter(DriveConstants.DRIVE_MAX_LINEAR_ACCELERATION);
    
    // Set default PID values for ROTATION_CONTROLLER
    ROTATION_CONTROLLER = new PIDController(DriveConstants.PROPORTIOAL_ROTATION_CONTROLLER,
        DriveConstants.INTEGRAL_ROTATION_CONTROLLER,
        DriveConstants.DIFFERENTIAL_ROTATION_CONTROLLER);

    // Add info to Shuffleboard
    ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add("Rotation PID", ROTATION_CONTROLLER);
    
    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);
  }
  
  @Override
  public void initialize()
  {
    targetHeading = SWERVE_SUBSYSTEM.getHeading();
  }
  
  // Running loop of command
  @Override
  public void execute()
  {
    // Set speeds based on Joystick values
    double xSpeed = -IOConstants.DRIVER_JOYSTICK_COMMAND_JOYSTICK.getRawAxis(IOConstants.JOYSTICK_X_AXIS_PORT);
    double ySpeed = IOConstants.DRIVER_JOYSTICK_COMMAND_JOYSTICK.getRawAxis(IOConstants.JOYSTICK_Y_AXIS_PORT);
    double turningSpeed = IOConstants.DRIVER_JOYSTICK_COMMAND_JOYSTICK.getRawAxis(IOConstants.JOYSTICK_TWIST_AXIS_PORT);
    
    // Check if joystick values are above deadzone
    if (Math.abs(xSpeed) < IOConstants.JOYSTICK_DEADZONE)
    {
      xSpeed = 0.0;
    }
    if (Math.abs(ySpeed) < IOConstants.JOYSTICK_DEADZONE)
    {
      ySpeed = 0.0;
    }
    if (Math.abs(turningSpeed) < IOConstants.JOYSTICK_DEADZONE)
    {
      turningSpeed = 0.0;
    }

    TURNING_SPEED_BEFORE_PID_COMPONENT.setDouble(turningSpeed);
    
    // Apply slew rate to joystick input to make robot input smoother and mulitply
    // by max speed
    xSpeed = X_LIMITER.calculate(xSpeed) * DriveConstants.DRIVE_MAX_LINEAR_SPEED;
    ySpeed = Y_LIMITER.calculate(ySpeed) * DriveConstants.DRIVE_MAX_LINEAR_SPEED;

    // Update speeds on Shuffleboard
    X_SPEED_COMPONENT.setDouble(xSpeed);
    Y_SPEED_COMPONENT.setDouble(ySpeed);
    
    // Set new heading
    targetHeading += turningSpeed;
    TARGET_HEADING_COMPONENT.setDouble(targetHeading);
    
    // Calculate turning speed required to reach desired heading
    turningSpeed = ROTATION_CONTROLLER.calculate(SWERVE_SUBSYSTEM.getHeading(), targetHeading);
    
    // If we are not at the turning minimum, don't turn.
    if (Math.abs(turningSpeed) < DriveConstants.TURNING_MINIMUM)
    {
      turningSpeed = 0.0;
    }
    
    // Limit turning speed
    if (Math.abs(turningSpeed) > DriveConstants.DRIVE_MAX_ANGULAR_SPEED)
    {
      turningSpeed = DriveConstants.DRIVE_MAX_ANGULAR_SPEED;
    }

    TURNING_SPEED_AFTER_PID_COMPONENT.setDouble(turningSpeed);
    
    // Set the module state
    // This sets the motor power for each Swerve Module
    // Joystick control is Field Relative, not Robot Relative
    SWERVE_SUBSYSTEM.setModuleStates(DriveConstants.DRIVE_KINEMATICS
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
            Rotation2d.fromDegrees(SWERVE_SUBSYSTEM.getRobotDegrees()))));
  }
  
  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted)
  {
    SWERVE_SUBSYSTEM.stopModules();
  }
  
  @Override
  public boolean isFinished()
  {
    return false;
  }
  
}

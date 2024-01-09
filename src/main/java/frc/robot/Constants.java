// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collection

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public final class Constants
{
  // Swerve modules
  public static final class ModuleConstants
  {
    // Drivetrain
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.14;
    public static final double TURNING_MOTOR_GEAR_RATIO = 1 / 12.8;
    
    // Encoder Conversion Factors
    public static final double DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSTION_FACTOR = DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR / 60;
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR / 60;
    
    // Turning PID
    public static final double TURNING_PROPORTIONAL = 0.5;
    public static final double TURNING_INTEGRAL = 0.0;
    public static final double TURNING_DIFFERENTIAL = 0.005;
  }
  
  // Swerve drive
  public static final class DriveConstants
  {
    // Distance between right and left wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.25);
    
    // Distance between front and back wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(21.25);
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
    
    // Driving Motor Ports
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
    
    // Turning Motor Ports
    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 4;
    public static final int BACK_LEFT_TURNING_MOTOR_ID = 8;
    public static final int BACK_RIGHT_TURNING_MOTOR_ID = 6;
    
    // Encoder on NEO turning
    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGHT_TURNING_ENCODING_REVERSED = true;
    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;
    
    // Encoder for NEO drive
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGH_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true;
    
    // Absolute Encoders
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 0;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 1;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 3;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 2;
    
    // Absolute encoders reversed
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    
    // Encoder Offsets
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 1.658063;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 1.39626;
    public static final double BACK_LEFT_ASOLUTE_ENCODER_OFFSET = 1.4;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = -0.122173;
    
    // Encoder Range
    public static final double ABSOLUTE_ENCODER_MINIMUM = 1.0 / 4096.0;
    public static final double ABSOLUTE_ENCODER_MAXIMUM = 4095.0 / 4096.0;
    
    // Max Speeds
    public static final double DRIVE_MAX_LINEAR_SPEED = 5;
    public static final double DRIVE_MAX_ANGULAR_SPEED = 2 * 2 * Math.PI;
    public static final double DRIVE_MAX_LINEAR_ACCELERATION = 8;
    public static final double DRIVE_MAX_ANGULAR_ACCELERATION = 2;
    
    // PID Thetas
    public static final double PROPORTIOAL_THETA_CONTROLLER = 0.001;
    public static final double INTEGRAL_THETA_CONTROLLER = 0.0;
    public static final double DIFFERENTIAL_THETA_CONTROLLER = 0.00;
    
    // Motor Current Limits
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20;
    
    // Misc
    public static final double TURNING_MINIMUM = 0.05;
  }
  
  // Input and Output
  public static final class IOConstants
  {
    public static final double JOYSTICK_DEADZONE = 0.05;
    
    private static final int JOYSTICK_PORT = 4;
    
    public static final Joystick DRIVER_JOYSTICK = new Joystick(JOYSTICK_PORT);
    
    public static final CommandJoystick DRIVER_JOYSTICK_COMMAND_JOYSTICK = new CommandJoystick(JOYSTICK_PORT);
  }
}

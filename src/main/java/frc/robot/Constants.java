// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collective

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
    // This value is multiplied by the raw encoder value to give the position
    public static final double DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER; 
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
    // This value is multiplied by the raw encoder value to give the velocity
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSTION_FACTOR = DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR / 60;
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR / 60;
    
    // Turning PID
    // See: https://en.wikipedia.org/wiki/Proportional–integral–derivative_controller
    public static final double TURNING_PROPORTIONAL = 0.05;
    public static final double TURNING_INTEGRAL = 0.0;
    public static final double TURNING_DIFFERENTIAL = 0.0;
  }
  
  // Swerve drive
  public static final class DriveConstants
  {
    // Distance between right and left wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.25);
    
    // Distance between front and back wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
    
    // Creates the Kinematics used in Swerve Drive
    // See javadoc for more info
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
    
    // Driving Motor Ports
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 12;
    
    // Turning Motor Ports
    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 22;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 23;
    public static final int BACK_LEFT_TURNING_MOTOR_ID = 21;
    public static final int BACK_RIGHT_TURNING_MOTOR_ID = 4;

    // Encoders on Drive Motors
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGH_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true;
    
    // Encoders on Turning Motors
    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_TURNING_ENCODING_REVERSED = false;
    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = false;
    
    // Absolute Encoders
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 32;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 33;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 31;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 30;
    
    // Absolute encoders reversed
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    
    // Encoder Offsets
    // These will need to me manually adjusted based on the individual Swerve Modules
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.0;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0.0;
    public static final double BACK_LEFT_ASOLUTE_ENCODER_OFFSET = 0.0;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0.0;
    
    // Max Speeds
    public static final double DRIVE_MAX_LINEAR_SPEED = .1;
    public static final double DRIVE_MAX_ANGULAR_SPEED = .1; 
    public static final double DRIVE_MAX_LINEAR_ACCELERATION = .1;
    public static final double DRIVE_MAX_ANGULAR_ACCELERATION = .1;
    
    // PID Thetas
    public static final double PROPORTIOAL_THETA_CONTROLLER = 0.05;
    public static final double INTEGRAL_THETA_CONTROLLER = 0.0;
    public static final double DIFFERENTIAL_THETA_CONTROLLER = 0.0;
    
    // Motor Current Limits

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 30;
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 30;
    
    // Misc
    public static final double TURNING_MINIMUM = 0.05; // Minimum turning speed which should actually cause a turn
    public static final double TURNING_SPEED_MULTIPLIER = 0.1; // Multiplier for the turning speed.
  }
  
  // Input and Output
  public static final class IOConstants
  {
    public static final double JOYSTICK_DEADZONE = 0.5;
    
    public static final int JOYSTICK_PORT = 0;
    public static final int JOYSTICK_X_AXIS_PORT = 0;
    public static final int JOYSTICK_Y_AXIS_PORT = 1;
    public static final int JOYSTICK_TWIST_AXIS_PORT = 3;
    
    public static final Joystick DRIVER_JOYSTICK = new Joystick(JOYSTICK_PORT);
    
    public static final CommandJoystick DRIVER_JOYSTICK_COMMAND_JOYSTICK = new CommandJoystick(JOYSTICK_PORT);
  }
}

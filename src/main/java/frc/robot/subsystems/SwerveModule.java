// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collective

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class SwerveModule extends SubsystemBase
{
  // Create empty variables for reassignment
  private final CANSparkMax DRIVE_MOTOR;
  private final CANSparkMax TURNING_MOTOR;
  
  private final RelativeEncoder DRIVE_ENCODER;
  private final RelativeEncoder TURNING_ENCODER;
  
  private final PIDController TURNING_PID_CONTROLLER;
  
  private final CANcoder MODULE_ENCODER;
  
  private final double MODULE_ENCODER_OFFSET_RAD;

  private final boolean ABSOLUTE_ENCODER_REVERSED;

  private final GenericEntry DRIVING_POSITION_SHUFFLEBOARD_COMPONENT;
  private final GenericEntry TURNING_POSITION_SHUFFLEBOARD_COMPONENT;
  private final GenericEntry MODULE_POSITION_SHUFFLEBOARD_COMPONENT;
  
  private final String MODULE_NAME;
  
  /**
   * Class constructor where we assign default values for variable
   * In swerve, there is no difference between the encoder being reversed and the motor being reversed; 
   * thus, this constructor takes in the encoder_reversed values and uses it to reverse the motors
  */ 
  public SwerveModule(int DRIVE_MOTOR_ID, int TURNING_MOTOR_ID, boolean DRIVE_MOTOR_REVERSED, boolean TURNING_MOTOR_REVERSED, 
    int ABSOLUTE_ENCODER_ID, double ABSOLUTE_ENCODER_OFFSET, boolean ABSOLUTE_ENCODER_REVERSED, String name)
  {
    // Set offsets for absolute encoder in radians
    MODULE_ENCODER_OFFSET_RAD = ABSOLUTE_ENCODER_OFFSET;
    
    MODULE_NAME = name;
    
    // Create absolute encoder
    MODULE_ENCODER = new CANcoder(ABSOLUTE_ENCODER_ID);
    this.ABSOLUTE_ENCODER_REVERSED = ABSOLUTE_ENCODER_REVERSED;

    // Create drive and turning motor
    DRIVE_MOTOR = new CANSparkMax(DRIVE_MOTOR_ID, MotorType.kBrushless);
    TURNING_MOTOR = new CANSparkMax(TURNING_MOTOR_ID, MotorType.kBrushless);
    
    // Set reverse state of drive and turning motor
    DRIVE_MOTOR.setInverted(DRIVE_MOTOR_REVERSED);
    TURNING_MOTOR.setInverted(TURNING_MOTOR_REVERSED);
    
    // Set drive and turning motor encoder values
    DRIVE_ENCODER = DRIVE_MOTOR.getEncoder();
    TURNING_ENCODER = TURNING_MOTOR.getEncoder();
    
    // Change drive motor conversion factors
    DRIVE_ENCODER.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR);
    DRIVE_ENCODER.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_CONVERSTION_FACTOR);
    
    // Change conversion factors for neo turning encoder - should be in radians!
    TURNING_ENCODER.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR);
    TURNING_ENCODER.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    
    // Create PID controller on ROBO RIO
    TURNING_PID_CONTROLLER = new PIDController(ModuleConstants.TURNING_PROPORTIONAL, ModuleConstants.TURNING_INTEGRAL, ModuleConstants.TURNING_DIFFERENTIAL);
    
    // Tell PID controller that it is a *wheel*
    TURNING_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    
    // Set motors to Brake when no input is sent
    DRIVE_MOTOR.setIdleMode(IdleMode.kBrake);
    TURNING_MOTOR.setIdleMode(IdleMode.kBrake);
    
    // Set current limit
    DRIVE_MOTOR.setSmartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    TURNING_MOTOR.setSmartCurrentLimit(DriveConstants.TURNING_MOTOR_CURRENT_LIMIT);
    
    // Call resetEncoders
    resetEncoders();

    // Add info to Shuffleboard
    DRIVING_POSITION_SHUFFLEBOARD_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add(name + " Position", getDrivePosition()).getEntry();
    TURNING_POSITION_SHUFFLEBOARD_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add(name + " Position", getTurningPosition()).getEntry();
    MODULE_POSITION_SHUFFLEBOARD_COMPONENT = ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add(name + " Position", MODULE_ENCODER.getAbsolutePosition().getValueAsDouble()).getEntry();
    ShuffleboardConstants.SWERVE_SHUFFLEBOARD_TAB.add(name + " Turning PID", TURNING_PID_CONTROLLER);
  }
  
  /** 
   * Updates values for the Module on Shuffleboard
   */
  public void update()
  {
    DRIVING_POSITION_SHUFFLEBOARD_COMPONENT.setDouble(getDrivePosition());
    TURNING_POSITION_SHUFFLEBOARD_COMPONENT.setDouble(getTurningPosition());
    MODULE_POSITION_SHUFFLEBOARD_COMPONENT.setDouble(MODULE_ENCODER.getAbsolutePosition().getValueAsDouble());
  }
  
  // Helpful get methods
  public double getDrivePosition()
  {
    return DRIVE_ENCODER.getPosition();
  }
  
  public double getTurningPosition()
  {
    return TURNING_ENCODER.getPosition();
  }
  
  public double getDriveVelocity()
  {
    return DRIVE_ENCODER.getVelocity();
  }
  
  public double getTurningVelocity()
  {
    return TURNING_ENCODER.getVelocity();
  }
  
  public SwerveModulePosition getPosition()
  {
    return (new SwerveModulePosition(
        getDrivePosition(), new Rotation2d(getTurningPosition())));
  }

  public String getName()
  {
    return MODULE_NAME;
  }
  
  /*
   * Convert absolute value of the encoder to radians and then subtract the radian
   * offset
   */
  public double getAbsoluteEncoderRad()
  {
    double angle = 0.0;
    
    // Get encoder absolute position goes from 1 to 0
    if (ABSOLUTE_ENCODER_REVERSED)
      angle = 1 - MODULE_ENCODER.getAbsolutePosition().getValueAsDouble();
    else
      angle = MODULE_ENCODER.getAbsolutePosition().getValueAsDouble();
    
    // Convert into radians
    angle *= 2.0 * Math.PI;

    // Apply offset
    angle -= MODULE_ENCODER_OFFSET_RAD; 
    
    return angle;
  }
  
  /**
   * Reset the position of the encoders
   * Turning motor encoder gets set to the module encoder absolute position
   */
  public void resetEncoders()
  {
    DRIVE_ENCODER.setPosition(0.0);
    TURNING_ENCODER.setPosition(MODULE_ENCODER.getAbsolutePosition().getValueAsDouble());
  }
  
  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  
  public void setDesiredState(SwerveModuleState state)
  {
    // Check if new command has high driving power
    if (Math.abs(state.speedMetersPerSecond) < 0.001)
    {
      stop();
      return;
    }
    
    // Optimize swerve module state to do fastest rotation movement, aka never
    // rotate more than 90*
    state = SwerveModuleState.optimize(state, getState().angle);
    
    // Scale velocity down using robot max speed
    DRIVE_MOTOR.set(state.speedMetersPerSecond * DriveConstants.DRIVE_MAX_LINEAR_SPEED);
    
    // Use PID to calculate angle setpoint
    TURNING_MOTOR.set(TURNING_PID_CONTROLLER.calculate(getTurningPosition(), state.angle.getRadians()));
  }
  
  // Stop all motors on module
  public void stop()
  {
    DRIVE_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }
  
  // Motor and SparkMax methods for Monitor
  public double[] getMotorsCurrent()
  {
    return (new double[] { DRIVE_MOTOR.getOutputCurrent(), TURNING_MOTOR.getOutputCurrent() });
  }
  
  public double[] getMotorsTemp()
  {
    return (new double[] { DRIVE_MOTOR.getMotorTemperature(), TURNING_MOTOR.getMotorTemperature() });
  }
  
  public void setSmartCurrentLimiter(int driveLimit, int turningLimit)
  {
    DRIVE_MOTOR.setSmartCurrentLimit(driveLimit);
    TURNING_MOTOR.setSmartCurrentLimit(driveLimit);
  }
}

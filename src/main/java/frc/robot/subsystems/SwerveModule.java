// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collective

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase
{
  // Create empty variables for reassignment
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  
  private final PIDController turningPidController;
  
  private final CANcoder absoluteEncoder;
  
  private final double absoluteEncoderOffsetRad;
  
  private String moduleName;
  
  // Class constructor where we assign default values for variable
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name)
  {
    // Set offsets for absolute encoder in radians
    absoluteEncoderOffsetRad = absoluteEncoderOffset;
    
    moduleName = name;
    
    SmartDashboard.putNumber(moduleName + " ABE Manual", 0);
    
    // Create absolute encoder
    absoluteEncoder = new CANcoder(absoluteEncoderId);

    //absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
    
    // Set duty cycle range of encoder of ABE encoder
    //absoluteEncoder.setDutyCycleRange(DriveConstants.ABSOLUTE_ENCODER_MINIMUM, DriveConstants.ABSOLUTE_ENCODER_MAXIMUM);
    
    // Create drive and turning motor
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
    
    // Set reverse state of drive and turning motor
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    
    // Set drive and turning motor encoder values
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
    
    // Change drive motor conversion factors
    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_CONVERSTION_FACTOR);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_CONVERSTION_FACTOR);
    
    // Change conversion factors for neo turning encoder - should be in radians!
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);
    
    // Create PID controller on ROBO RIO
    turningPidController = new PIDController(ModuleConstants.TURNING_PROPORTIONAL, ModuleConstants.TURNING_INTEGRAL, ModuleConstants.TURNING_DIFFERENTIAL);
    
    // Tell PID controller that it is a *wheel*
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Set motors to Brake when no input is sent
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);
    
    // Set current limit
    driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turningMotor.setSmartCurrentLimit(DriveConstants.TURNING_MOTOR_CURRENT_LIMIT);
    
    // Call resetEncoders
    resetEncoders();
  }
  
  public void update()
  {
    SmartDashboard.putNumber(moduleName + "Absolute-Position", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }
  
  // Helpful get methods
  public double getDrivePosition()
  {
    return driveEncoder.getPosition();
  }
  
  public double getTurningPosition()
  {
    return turningEncoder.getPosition();
  }
  
  public double getDriveVelocity()
  {
    return driveEncoder.getVelocity();
  }
  
  public double getTurningVelocity()
  {
    return turningEncoder.getVelocity();
  }
  
  public SwerveModulePosition getPosition()
  {
    return (new SwerveModulePosition(
        getDrivePosition(), new Rotation2d(getTurningPosition())));
  }
  
  /*
   * Convert absolute value of the encoder to radians and then subtract the radian
   * offset
   * then check if the encoder is reversed.
   */
  public double getAbsoluteEncoderRad()
  {
    double angle;
    
    // Get encoder absolute position goes from 1 to 0
    angle = 1 - absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    
    // Convert into radians
    angle *= 2.0 * Math.PI;

    // Apply offset
    angle -= absoluteEncoderOffsetRad; 
    
    return angle;
  }
  
  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders()
  {
    driveEncoder.setPosition(0);
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
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.DRIVE_MAX_LINEAR_SPEED);
    
    // Use PID to calculate angle setpoint
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  }
  
  // Stop all motors on module
  public void stop()
  {
    driveMotor.set(0);
    turningMotor.set(0);
  }
  
  // Motor and SparkMax methods for Monitor
  public double[] getMotorsCurrent()
  {
    return (new double[] { driveMotor.getOutputCurrent(), turningMotor.getOutputCurrent() });
  }
  
  public double[] getMotorsTemp()
  {
    return (new double[] { driveMotor.getMotorTemperature(), turningMotor.getMotorTemperature() });
  }
  
  public void setSmartCurrentLimiter(int driveLimit, int turningLimit)
  {
    driveMotor.setSmartCurrentLimit(driveLimit);
    turningMotor.setSmartCurrentLimit(driveLimit);
  }
}

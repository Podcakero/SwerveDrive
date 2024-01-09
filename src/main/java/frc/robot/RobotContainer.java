// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org
// Modified by Greyson Weimer for FRC 2890 The Hawk Collection

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer
{
  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  SendableChooser<Command> chooser = new SendableChooser<>();
  
  public RobotContainer()
  {
    SmartDashboard.putData(chooser);
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem));
  }
  
  // Return the command to run during auto
  public Command getAutonomousCommand()
  {
    return chooser.getSelected();
  }
}
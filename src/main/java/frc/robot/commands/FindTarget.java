// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLightVision;

public class FindTarget extends Command 
{
  private DriveTrain drivetrain_;
  private LimeLightVision vision_;
  private Boolean visionTargetFound_;

  /** Creates a new FindTarget. */
  public FindTarget( DriveTrain drivetrain, 
                           LimeLightVision vision )
  {
    drivetrain_ = drivetrain;
    vision_ = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision_, drivetrain_);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Running FindTarget Command");
    visionTargetFound_ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Rotate robot slowly CCW
    drivetrain_.arcadeDrive(0, 0.5);

    // Check if we see an Apriltag.
    visionTargetFound_ = vision_.isValidVisionTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending FindTarget Command");
    drivetrain_.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if ( visionTargetFound_ == true )
      return true;
    else
      return false;
  }
}

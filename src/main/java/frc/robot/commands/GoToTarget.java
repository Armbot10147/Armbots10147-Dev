// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.Constants;

public class GoToTarget extends Command 
{
  private DriveTrain drivetrain_;
  private double angular_kP = Constants.limelightConstants.angular_kP; // Largest angle=31, 1/31=0.032
  private double linear_kP = Constants.limelightConstants.linear_kP;
  private double currentDistanceToTargetInches_;
  private double goalToTargetInches_;
  private LimeLightVision vision_;

  /** Creates a new GoToVisionTarget. */
  public GoToTarget( DriveTrain drivetrain, 
                           double goalToTargetInches,
                           LimeLightVision vision ) 
  {
    drivetrain_ = drivetrain;
    goalToTargetInches_ = goalToTargetInches;
    vision_ = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision_, drivetrain_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Running GoToTarget Command");
    //table = NetworkTableInstance.getDefault().getTable("limelight");
    // Reset encoders to 0.
    //drivetrain_.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Get angle to target, tx (-31degrees to +31degrees) represents error in "Proportional".
    // Convert error to an angular velocity command, use angular_kP.  Note, error is
    // simply the angle to target since the setpoint is 0.
    if ( vision_.isValidVisionTarget() )
    {
      // 1. Angle to target.
      double angleToTargetDegrees = vision_.getTX();
      SmartDashboard.putNumber("Apriltag angle(degrees):", angleToTargetDegrees );

      double angularVelToTarget = angleToTargetDegrees * angular_kP ; // * kMaxAngularSpeed;
      angularVelToTarget *= -1.0; // CCW is positive, limelight w/target to left is negative.

      // 2. Distance to target
      currentDistanceToTargetInches_ = vision_.visionTargetDistance();
      SmartDashboard.putNumber("Apriltag distance(inches):", currentDistanceToTargetInches_ );

      double error = currentDistanceToTargetInches_ - goalToTargetInches_;
      SmartDashboard.putNumber("Apriltag distance error(inches):", error );
      double linearVelToTarget = error * linear_kP;

      //  Not sure if this is necessary.
      //if (linearVelToTarget > 1.0 ) 
      //  linearVelToTarget = 1.0;
      //if (angularVelToTarget < -1.0) 
      //  angularVelToTarget = -1.0;
      //else if (angularVelToTarget > 1.0)
      //  angularVelToTarget = 1.0;
      // Command drivetrain
      drivetrain_.arcadeDrive(linearVelToTarget, angularVelToTarget);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending GoToTarget Command");
    drivetrain_.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // Need to determine how close we need to get to the Apriltag.
    if (currentDistanceToTargetInches_ < goalToTargetInches_ ) 
      return true;
    else
      return false;
  }
}

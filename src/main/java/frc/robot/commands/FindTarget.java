package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLightVision;
import edu.wpi.first.wpilibj2.command.Command;

public class FindTarget extends Command {

    private final DriveTrain driveTrain_;
    private final LimeLightVision limelight_;
    private boolean visionTargetFound_;

    public FindTarget(DriveTrain driveTrain, LimeLightVision limelight) {
        driveTrain_ = driveTrain;
        limelight_ = limelight;

        addRequirements(driveTrain_, limelight_);
    }

    @Override
    public void initialize() {
        visionTargetFound_ = false;
    }

    @Override
    public void execute() {
        findAprilTag();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain_.arcadeDrive(0, 0); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        return visionTargetFound_;
    }

    private void findAprilTag() {
        double kP_turn = 0.032; // Proportional control constant for turning

        while (limelight_.isValidVisionTarget()) { // Check if a valid target is detected
            double tx = limelight_.getTX(); // Get horizontal offset from Limelight

            // Calculate turning speed using proportional control
            double turnSpeed = kP_turn * tx;

            // Clamp the turn speed to prevent excessive rotation
            turnSpeed = Math.max(-1.0, Math.min(1.0, turnSpeed));

            // Turn the robot to align with the target
            driveTrain_.arcadeDrive(0, -turnSpeed); // Negative to correct the rotation direction

            // Check if aligned within a small tolerance
            if (Math.abs(tx) < 1.0) { // Tolerance of 1 degree
                visionTargetFound_ = true;
                break;
            }

            // Add a small delay to avoid CPU overloading
            try {
                Thread.sleep(20); // 20 ms delay
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Stop the robot after alignment
        driveTrain_.arcadeDrive(0, 0);
    }
}
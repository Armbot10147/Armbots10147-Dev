package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;

public class Robot_Functions extends SubsystemBase {
    private final DriveTrain driveTrain;
    private final Timer timer;
    private final double speed;

    public Robot_Functions(DriveTrain driveTrain, Timer timer, double speed) {
        this.driveTrain = driveTrain;
        this.timer = timer;
        this.speed = speed;
    }

    // Move the robot forward for a given amount of time (seconds)
    public void moveForward(double seconds) {
        timer.start();
        while (timer.hasElapsed(seconds) == false) {
            driveTrain.tankDrive(speed, speed); // Move forward at the specified speed
        }
        timer.stop();
        timer.reset();
        driveTrain.stop(); // Stop motors after moving
    }

    // Move the robot backward for a given amount of time (seconds)
    public void moveBackward(double seconds) {
        timer.start();
        while (timer.hasElapsed(seconds) == false) {
            driveTrain.tankDrive(-speed, -speed); // Move backward at the specified speed
        }
        timer.stop();
        timer.reset();
        driveTrain.stop(); // Stop motors after moving
    }

    // Turn the robot right for a given amount of time (seconds)
    public void turnRight(double seconds) {
        timer.start();
        while (timer.hasElapsed(seconds) == false) {
            driveTrain.tankDrive(speed, -speed); // Turn right
        }
        timer.stop();
        timer.reset();
        driveTrain.stop(); // Stop motors after turning
    }

    // Turn the robot left for a given amount of time (seconds)
    public void turnLeft(double seconds) {
        timer.start();
        while (timer.hasElapsed(seconds) == false) {
            driveTrain.tankDrive(-speed, speed); // Turn left
        }
        timer.stop();
        timer.reset();
        driveTrain.stop(); // Stop motors after turning
    }
}

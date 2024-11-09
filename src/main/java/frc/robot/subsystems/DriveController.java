package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveController {
    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;
    private Encoder encoder;
    
    private final double kP = 0.5;
    private final double kI = 0.5;
    private final double kD = 0.1;
    private final double iLimit = 1;
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    private double errorSum = 0;
    private double lastTimestamp = 0;
    private double lastError = 0;

    public DriveController(WPI_TalonSRX leftMotor, WPI_TalonSRX rightMotor, Encoder encoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.encoder = encoder;
    }

    public void reset() {
        encoder.reset();
        errorSum = 0;
        lastError = 0;
        lastTimestamp = Timer.getFPGATimestamp();
    }

    public void driveToPosition(double setpoint) {
        double sensorPosition = encoder.get() * kDriveTick2Feet;
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

        // Output to motors without inverting
        leftMotor.set(outputSpeed);
        rightMotor.set(outputSpeed);

        // Update last variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
    }

    // Getter for kDriveTick2Feet
    public double getDriveTick2Feet() {
        return kDriveTick2Feet;
    }
}
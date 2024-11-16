package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX rightFrontMotor;
    private final WPI_TalonSRX rightBackMotor;
    private final WPI_TalonSRX leftFrontMotor;
    private final WPI_TalonSRX leftBackMotor;

    private final DifferentialDrive differentialDrive;

    public static final double kMaxAngularSpeed = Math.PI;
    public static final double kMaxSpeed = 3.0;

    public DriveTrain() {
        // Initialize motors with the new CAN IDs
        rightFrontMotor = new WPI_TalonSRX(1); // Right Forward
        rightBackMotor = new WPI_TalonSRX(2);  // Right Back
        leftFrontMotor = new WPI_TalonSRX(3);  // Left Forward
        leftBackMotor = new WPI_TalonSRX(4);   // Left Back

        // Set back motors to follow the front motors
        rightBackMotor.follow(rightFrontMotor);
        leftBackMotor.follow(leftFrontMotor);

        // Set motor inversion if necessary
        rightFrontMotor.setInverted(true); // Invert the right side motors
        rightBackMotor.setInverted(InvertType.FollowMaster); // Follow the inversion of the master
        leftFrontMotor.setInverted(false); // No inversion on the left side
        leftBackMotor.setInverted(InvertType.FollowMaster); // Follow the inversion of the master

        // Create the differential drive
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    }

    // Methods to control the drivetrain
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
        differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed);
    }

    public void stop() {
        differentialDrive.stopMotor();
    }
}

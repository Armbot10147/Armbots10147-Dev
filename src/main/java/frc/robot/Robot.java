package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot_Functions;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

    private DriveTrain driveTrain;
    private Robot_Functions robotFunctions;
    private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
    private XboxController joystick;
    private Timer timer;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public interface PS4 {
        int BLUE_X = 1;
        int RED_CIRCLE = 2;
        int PINK_SQUARE = 3;
        int GREEN_TRIANGLE = 4;
        int LEFT_BUMPER = 5;
        int RIGHT_BUMPER = 6;
        int SHARE_BUTTON = 7;
        int OPTIONS_BUTTON = 8;
        int LEFT_JOYSTICK_PRESS = 9;
        int RIGHT_JOYSTICK_PRESS = 10;
        
        int LEFT_JOYSTICK_HORIZONTAL = 0;
        int LEFT_JOYSTICK_VERTICAL = 1;
        int LEFT_TRIGGER_LOWER = 2;
        int RIGHT_TRIGGER_LOWER = 3;
        int RIGHT_JOYSTICK_HORIZONTAL = 4;
        int RIGHT_JOYSTICK_VERTICAL = 5;
    }

    public interface XBOX {
        int GREEN_A = 1;
        int RED_B = 2;
        int BLUE_X = 3;
        int YELLOW_Y = 4;

        int LEFT_BUMPER = 5;
        int RIGHT_BUMPER = 6;
        int SCRRENSHOT_BUTTON = 7;
        int MENU_BUTTON = 8;

        int L3 = 9;
        int R3 = 10;
    }

    // Robot initialization
    @Override
    public void robotInit() {
        // Initialize the joystick on USB port 0
        joystick = new XboxController(0);

        // Initialize timer
        timer = new Timer();

        // Initialize the DriveTrainSubsystem
        driveTrain = new DriveTrain();

        // Initialize Robot_Functions with the new DriveTrainSubsystem
        robotFunctions = new Robot_Functions(driveTrain, timer, 0.5); // Example speed: 0.5
    }

    @Override
    public void autonomousInit() {
        driveTrain.stop();
    }

    @Override
    public void autonomousPeriodic() {
        // Autonomous actions based on joystick input
        if (joystick.getRawButton(XBOX.RIGHT_BUMPER)) {
            SmartDashboard.putString("Autonomous Status", "Moving 10 feet");
            driveTrain.tankDrive(0.5, 0.5); // Move forward at 50% speed
        } else if (joystick.getRawButton(XBOX.LEFT_BUMPER)) {
            SmartDashboard.putString("Autonomous Status", "Moving back to 0");
            driveTrain.tankDrive(-0.5, -0.5); // Move backward at 50% speed
        } else {
            driveTrain.stop();
        }
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
    }

    @Override
    public void teleopPeriodic() {
        // Example joystick control with Robot_Functions
        double moveSeconds = 0.4;
        double turnSeconds = 0.1;

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
            
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        double rightTrigger = joystick.getRightTriggerAxis();
        double leftTrigger = joystick.getLeftTriggerAxis();

        double forward = rightTrigger-leftTrigger;
        double rotation = -joystick.getLeftX()*0.5;

        driveTrain.arcadeDrive(forward, rotation);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // Check for specific button inputs to trigger robot functions
        if (joystick.getRawButtonPressed(XBOX.YELLOW_Y)) {
            robotFunctions.moveForward(moveSeconds);
        }
        if (joystick.getRawButtonPressed(XBOX.GREEN_A)) {
            robotFunctions.moveBackward(moveSeconds);
        }
        if (joystick.getRawButtonPressed(XBOX.RED_B)) {
            robotFunctions.turnRight(turnSeconds);
        }
        if (joystick.getRawButtonPressed(XBOX.BLUE_X)) {
            robotFunctions.turnLeft(turnSeconds);
        }

        // Generic button press detection
        for (int buttonID = 1; buttonID <= 10; buttonID++) {
            if (joystick.getRawButton(buttonID)) {
                System.out.println("Button pressed: " + buttonID);
            }
        }
    }
}
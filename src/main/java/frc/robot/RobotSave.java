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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotSave extends TimedRobot {

    private DriveTrain driveTrain;
    private Robot_Functions robotFunctions;
    private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
    private XboxController joystick;
    private Timer timer;

    private RobotContainer m_robotContainer;
    //private SlewRateLimiter linear_limiter = new SlewRateLimiter(1);
    //private SlewRateLimiter angular_limiter = new SlewRateLimiter(0.8);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public void followAprilTag() {
        double kP_turn = 0.032; // Tuning parameter for turning control
        double kP_drive = 0.1; // Tuning parameter for forward/backward drive control
    
        // Retrieve target offset angles from the Limelight
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0.0); // Horizontal offset from crosshair to target (-27 to 27 degrees)
        double ty = table.getEntry("ty").getDouble(0.0); // Vertical offset from crosshair to target
        double ta = table.getEntry("ta").getDouble(0.0); // Target area (not used but available)
        double tagID = table.getEntry("tid").getDouble(-1); // Retrieved tag ID, if needed

        // Calculate turning speed using a proportional control loop
        double AngularVelocity = kP_turn * tx;
        //AngularVelocity *= driveTrain.kMaxAngularSpeed;
        AngularVelocity *=-1.0;
    
        // Calculate forward speed using ty for simple distance control
        double forwardSpeed = kP_drive * ty;
        forwardSpeed *= driveTrain.kMaxSpeed;
        forwardSpeed *= -1;
    
        // Adjust driveSpeed and turnSpeed as per desired behavior
        // Drive towards the target using arcade drive control
        driveTrain.arcadeDrive(0, AngularVelocity); // Negative sign on driveSpeed if you need to invert direction
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
        NetworkTableEntry tid = table.getEntry("tid");
            
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double tagID = tid.getDouble(0);

        double rightTrigger = joystick.getRightTriggerAxis()*0.7;
        double leftTrigger = joystick.getLeftTriggerAxis()*0.7;

        double forward = rightTrigger-leftTrigger;
        double rotation = -joystick.getLeftX()*0.5;

        driveTrain.arcadeDrive(forward, rotation);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("CurrentID", tagID);

        if (joystick.getAButton()) {
            followAprilTag();
        }

        // Generic button press detection
        //for (int buttonID = 1; buttonID <= 10; buttonID++) {
        //    if (joystick.getRawButton(buttonID)) {
        //        System.out.println("Button pressed: " + buttonID);
        //    }
        //}
    }
}
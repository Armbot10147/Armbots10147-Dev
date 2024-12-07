package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.*;

public class Robot extends TimedRobot {

    private RobotContainer m_robotcontainer;
    private Command m_autonomousCommand;

    // Robot initialization
    @Override
    public void robotInit() {
        m_robotcontainer = new RobotContainer();
    }

    // Periodic function called during teleop mode
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotcontainer.getAutonomousCommand();

        if (m_robotcontainer != null){
            m_autonomousCommand.schedule();
        }
    }
}

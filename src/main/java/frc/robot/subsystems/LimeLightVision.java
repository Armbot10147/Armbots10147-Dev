package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelightConstants;

public class LimeLightVision extends SubsystemBase{
    private final NetworkTable limelightTable;
    private double tx, ty, ta, tv;
    private double distanceToTargetInches;

    public LimeLightVision(){
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic(){
        tx = limelightTable.getEntry("tx").getDouble(0.0);
        ty = limelightTable.getEntry("ty").getDouble(0.0);
        ta = limelightTable.getEntry("ta").getDouble(0.0);
        tv = limelightTable.getEntry("tv").getDouble(0.0);
    }

    /*Returns the tx value given from the limelight */
    public double getTX(){
        return tx;
    }

    /*Returns the ty value given from the limelight */
    public double getTY(){
        return ty;
    }

    /*Returns the ta value given from the limelight */
    public double getTA(){
        return ta;
    }

    /*Returns the tv value given from the limelight */
    public double getTV(){
        return tv;
    }

    /*Checks whether or not there is a target being detected and return a boolean of true or false */
    public Boolean isValidVisionTarget(){
        SmartDashboard.putNumber("TV value", tv);
        if (tv<0.1){
            return false;
        }
        else{
            return true;
        }
    }

    /*
     */
    public double visionTargetDistance(){

        double targetAngle_Vertical = ty;
        double verticalAngleToTargetDegrees = limelightConstants.limelightMountAngleDegrees + targetAngle_Vertical;
        double verticalAngleToTargetRadians = verticalAngleToTargetDegrees * Math.PI / 180.0;

        distanceToTargetInches = (limelightConstants.targetHeightInches - limelightConstants.limelightCameraHeightInches) / Math.tan(verticalAngleToTargetRadians);
        return distanceToTargetInches;
    }
}

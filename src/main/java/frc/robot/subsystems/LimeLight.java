package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;

public class LimeLight extends SubsystemBase {

    // private LED led = new LED();
    //setup networktable upon creation
    private NetworkTable nTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = nTable.getEntry("tx");
    private NetworkTableEntry ty = nTable.getEntry("ty");
    private NetworkTableEntry ta = nTable.getEntry("ta");
    private NetworkTableEntry tv = nTable.getEntry("tv");
    private NetworkTableEntry tid = nTable.getEntry("tid");
    //Values posted by the limelight.
    private double currentX; // X value is horizontal angle from center of LL camera
    private double currentY; // Y value is vertical angle from center of LL camera
    private double currentArea; // Unknown what this does currently.
    private double seesTarget; //Double value (only 1 or 0) that tells the program if it sees the target.
    private double curTargetID; //Double value designating the current visible AprilTag.
    //Values for posting to Shuffleboard
    private double estimDist = 0.0;
    private double showTurnPower = 0.0;

    //BOOLEANS
    private boolean enabled = false;
    private boolean targetFound = false;

    //TIMERS
    private final Timer seekTimer = new Timer();
    private final Timer driveTimer = new Timer();
    private final Timer refreshTimer = new Timer();

    //HEIGHTS                                     ID 1   ID 2 ID 3 ID 4 ID 5 ID 6    ID 7 
    //ID #s are the number minus 1.
    private final double[] heightArray = {50.125,50.125,53.88, 0.0, 0.0, 0.0, 0.0, 50.125, 53.88};

    //CONSTANTS
    //Physical distance of limelight LENS from ground (measured in INCHES)
    private final double LensDistFromGround = 8.5;
    //Physical vertical angle of lens from mount (measured in DEGREES).
    private final double LensAngleFromMount = 37.5;
    //Physical height of chosen AprilTag.;
    //Correction modifier. I assume it designates how much of a correction you want.
    private final double correctionMod = -.1;
    //Preset distance from target.
    //Could put it in an array and designate it to an AprilTag.
    private final double desiredDist = 36.25;

    //#LIMELIGHT
    /* Constructor. Assigns values to the coordinate variables a%bove.
    */
    public LimeLight()
    {
        //Start catching limelight values
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);
        curTargetID = tid.getDouble(0.0);

        //Make them visible (via SmartDashboard)
        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
        SmartDashboard.putNumber("DistFromTarget", estimDist);
        SmartDashboard.putNumber("VisibleTargetID", this.curTargetID);

        start();
    }



    //#ESTIMATEDIST
    /* Does math to estimate the distance from the limelight to the target.
        Assumes that robot is centered on target.
     */
    private double targetHeight = 0.0;

    public double estimateDist()
    {
        int tempID = (int) curTargetID;

        if (tempID > 0)
        {
            targetHeight = heightArray[tempID];
        }

        double radAngle = Math.toRadians(this.currentY + LensAngleFromMount);

        //Simple trigonometry to return distance from given angle 
        double distFromGoal = ((targetHeight - LensDistFromGround) / Math.tan(radAngle));
        estimDist = distFromGoal;
        return distFromGoal;
    }



    //#STOP
    /* Force-Stops all limelight functionality.
     * Used whenever the time-out limit is reached to prevent penalties.
    */
    public void stop()
    {
        seekTimer.stop();
        seekTimer.reset();
        driveTimer.stop();
        driveTimer.reset();
        refreshTimer.stop();
        refreshTimer.reset();
        enabled = false;
        targetFound = false;
    }



    //#START
    /*Enables the limelight.
     * Allows for multi-use of autonomous.
     */
    public void start()
    {
        stop();
        
        enabled = true;
        targetFound = false;
    }



    //#GETINRANGEUSINGAREA
    /*An alternative to getInRangeUsingDistance. 
     * Approaches the target using the total area taken rather than a set distance.
     * Used for getting rough approaches.
     */
    public void getInRangeUsingArea(DriveTrain driveTrain)
    {
        if(driveTimer.get() > 3.0 && seesTarget == 0.0)
        {
            driveTrain.HamsterDrive.arcadeDrive(0, 0);
            stop();
        }
        if (enabled)
        {
            if (driveTimer.get() == 0.0 && targetFound) {driveTimer.start(); refreshTimer.start();}

            if (driveTimer.get() > 0.0)
            {
                if (currentArea <= 15.0)
                {
                    double speed = -.35;
                    driveTrain.HamsterDrive.arcadeDrive(speed, 0);
                } 
                else 
                {
                    driveTrain.HamsterDrive.arcadeDrive(0, 0);
                    stop();
                }
            }
        }  
    }



    //#GETINRANGEUSINGDISTANCE
    /* Allows the robot to precisely get in range of a target.
     * THEORETICALLY should work.
     */
    private double speed = -.45;
    public void getInRangeUsingDistance(DriveTrain driveTrain)
    {
        if(driveTimer.get() > 3.0 && refreshTimer.get() > 3.0)
        {
            driveTrain.drive(0, 0);
            stop();
            // led.setBoard("red");
        }
        if (enabled)
        {
            if (driveTimer.get() == 0.0 && targetFound) {driveTimer.start(); refreshTimer.start();}

            if (driveTimer.get() > 0.0)
            {
                if(seesTarget != 0.0)
                {
                    refreshTimer.reset();
                    refreshTimer.start();
                }

                //Estimate distance from target and the distance error.
                double currentDist = estimateDist();
                double distError = desiredDist - currentDist; //Distance from desired point. Calculated in Inches.

                if (distError > 3 || distError < -3)
                {
                    // led.setBoard("blue");

                    //Calculate driving adjust percentage for turning.
                    double drivingAdjust  = ((correctionMod * distError) * .1); //% of angle (i think)

                    //Cap the speed at 45%
                    if (drivingAdjust > 0) 
                    {
                        speed = .325;
                    }
                    else if (drivingAdjust < 0)
                    {
                        speed = -.325;
                    }

                    double turnPower = Math.pow((this.currentX*.1), 3);

                    //Cap turn power at 35% of value
                    if (turnPower < -.35) 
                    {
                        turnPower = -.35;
                    }
                    else if (turnPower > .35) 
                    {
                        turnPower = .35;
                    }

                    showTurnPower = turnPower;
                    driveTrain.drive(speed, turnPower);

                } 
                else 
                {
                    // led.setBoard("green");
                    driveTrain.drive(0, 0);
                    stop();
                }
            }
        }  
    }



    /*#SEEKTARGET
     * Turns the robot until the limelight catches a glimpse of the target
     * On the robot seeing it, centers on the target with a .5 degree range of error.
     * Unknown which way the directions are.
     */
    private double steeringPow = .25;

    public boolean seekTarget(DriveTrain driveTrain)
    {
        if (seekTimer.get() > 10.0 && seesTarget == 0.0)
        {
            driveTrain.drive(0, 0);
            stop();
            targetFound = false;
            // led.setBoard("red");
        }
        if (enabled && !targetFound) 
        {
           if (seekTimer.get() <= 0.0) seekTimer.start();

           if (seekTimer.get() > 0.0)
           {
            //If target isn't in view, set steeringPow to be a consistent .3. 
                if (seesTarget == 0.0)
                {
                    steeringPow = .25;
                    driveTrain.drive(0, steeringPow);
                    // led.setBoard("blue");
                } 
                else 
                {
                    //Else if it is visible then...
                    //Runs if it is not in the threshold.
                    if ((currentX > 5.0 || currentX < -5.0) && seesTarget != 0.0)
                    {
                        if (currentX > 0.0) 
                        {
                            steeringPow = .275;
                        }
                        else if (currentX < 0.0) 
                        {
                            steeringPow = -.275;
                        }

                        driveTrain.drive(0, steeringPow);

                    } 
                    else if ((currentX < 5.0 && currentX > -5.0) && seesTarget != 0.0)
                    {
                        //We have found the target. Stop turning.
                        driveTrain.drive(0, 0);
                        seekTimer.stop();
                        seekTimer.reset();
                        targetFound = true;
                        // led.setBoard("green");
                    }
                }
           }
        }
        return targetFound;
    }



    //#POSTVALUES
    /* Post values from the limelight to variables, then relays them to SmartDashboard for human viewing. 
    */
    public void postValues()
    {
        //Get values from the limelight.
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentArea = ta.getDouble(0.0);
        seesTarget = tv.getDouble(0.0);
        curTargetID = tid.getDouble(0.0);

        //Post SmartDashboard values
        SmartDashboard.putNumber("LimelightX", this.currentX);
        SmartDashboard.putNumber("LimelightY", this.currentY);
        SmartDashboard.putNumber("LimelightArea", this.currentArea);
        SmartDashboard.putNumber("LimeLightSeesTarget", this.seesTarget);
        SmartDashboard.putNumber("DistFromTarget", estimDist);
        SmartDashboard.putNumber("VisibleTargetID", this.curTargetID);
        SmartDashboard.putNumber("TurnPowerAdjust", showTurnPower);
    }

    public boolean getEnabled(){
        return !enabled;
    }

    public void runLimelight (DriveTrain drivetrain) 
    {
        if(seekTarget(drivetrain)) getInRangeUsingDistance(drivetrain);
    }

    @Override
    public void periodic(){
        postValues();
    }
}

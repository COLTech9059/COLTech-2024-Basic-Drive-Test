package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Manipulator 
{
    
    //Create the LED class object
    // private LED led = new LED();

    //Create the motor controller objects
    CANSparkMax leftBaseMotor = new CANSparkMax(Constants.leftBaseID, MotorType.kBrushless);
    CANSparkMax rightBaseMotor = new CANSparkMax(Constants.rightBaseID, MotorType.kBrushless);
    CANSparkMax ampMotor = new CANSparkMax(Constants.ampID, MotorType.kBrushless);
    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);
    CANSparkMax followerAmpMotor = new CANSparkMax(Constants.ampFollowID, MotorType.kBrushless);

    //Create the encoder objects
    RelativeEncoder leftBaseEncoder = leftBaseMotor.getEncoder();
    RelativeEncoder rightBaseEncoder = rightBaseMotor.getEncoder();
    RelativeEncoder ampEncoder = ampMotor.getEncoder();
    RelativeEncoder leftIntakeEncoder= followerAmpMotor.getEncoder();
    RelativeEncoder rightIntakeEncoder = intakeMotor.getEncoder();

    //Create the digital input objects
    static DigitalInput intakeSensor = new DigitalInput(0);

    //Reading true = Sensor not triggered, reading false = Sensor is triggered
    DigitalInput frontSensor = new DigitalInput(1); 
    DigitalInput backSensor = new DigitalInput(2);

    private boolean usingDigitalSensors = true;

    //#INITIALIZEMANIPULATOR
    //This method will set up the manipulator for use
    public void initializeManipulator() 
    {

        //Reset the digital sensors boolean
        usingDigitalSensors = true;

        //Reset intake boolean
        // if (!beamSensor.get()) {didIntake = false;} else {didIntake = true;}

        //Reset the motors to their factory defaults
        leftBaseMotor.restoreFactoryDefaults();
        rightBaseMotor.restoreFactoryDefaults();
        ampMotor.restoreFactoryDefaults();
        followerAmpMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        //Set the leftIntakeMotor as a follower
        followerAmpMotor.follow(ampMotor);

        //Set the leftBaseMotor as a follower
        leftBaseMotor.follow(rightBaseMotor);

        //Set the encoders to 0, effectively resetting them
        leftBaseEncoder.setPosition(0);
        rightBaseEncoder.setPosition(0);
        ampEncoder.setPosition(0);
        rightIntakeEncoder.setPosition(0);
        leftIntakeEncoder.setPosition(0);

        rightBaseMotor.setOpenLoopRampRate(0.25);
    }



    //#manualControl
    //This method toggles the use of digital sensors duting teleop
    public void manualControl() 
    {
        if (usingDigitalSensors) {usingDigitalSensors = false;} else {usingDigitalSensors = true;}
    }

    
    private static boolean didIntakePosition = false;
    private static Timer posTimer = new Timer();

    //#INTAKEPOSITION
    //This method will run the manipulator base motors until the magnetic sensor is triggered at the amp spitting position
    /*
     * @Param timeout               The time limit for the method
     * @Return didIntakePosition    Returns true if successfully positioned, and false if not
     */
    public boolean intakePosition(double timeout) 
    {
        posTimer.reset();
        posTimer.start();

        if (posTimer.get() <= timeout) 
        {
            if (frontSensor.get()) 
            {
            rightBaseMotor.set(0.3);
            } 
            else if (!frontSensor.get()) 
            {
            rightBaseMotor.set(0);
            rightBaseEncoder.setPosition(0);
            leftBaseEncoder.setPosition(0);

            didIntakePosition = true;
            // led.setBoard("green");
            posTimer.stop();
            posTimer.reset();
            }
        } 
        else 
        {
        rightBaseMotor.set(0);
        didIntakePosition = false;
        // led.setBoard("red");
        }
        return didIntakePosition;
    }


    //#SHOOTPOSITION
    //This method brings the robot to the shooting position
    public void shootPosition() 
    {
        if (rightBaseEncoder.getPosition() > Constants.shootPosition + 0.5)
        {
            rightBaseMotor.set(-0.3);
        }
        if (rightBaseEncoder.getPosition() < Constants.shootPosition - 1)
        {
            rightBaseMotor.set(0.15);
        }
    }
    
    private static Timer shootPosTime = new Timer();
    private boolean didShootPosition = false;

    //#SHOOTPOSITION
    //This method will bring the manipulator to a position for it to shoot from
    /*
     * @Param timeout            The time limit for the method
     * @Return didShootPosition  Returns true if successfully positioned, returns false if not
     */
    public boolean shootPosition(double timeout) 
    {
        shootPosTime.reset();
        shootPosTime.start();

        if (shootPosTime.get() <= timeout) 
        {
            if (rightBaseEncoder.getPosition() < Constants.shootPosition) 
            {
                rightBaseMotor.set(0.3);
            } 
            else if (rightBaseEncoder.getPosition() > Constants.shootPosition)
            {
                rightBaseMotor.set(-0.3);
            } 
            else if (rightBaseEncoder.getPosition() >= Constants.shootPosition + 0.5 && rightBaseEncoder.getPosition() <= Constants.shootPosition - 0.5)
            {
                rightBaseMotor.set(0);
                didShootPosition = true;
                // led.setBoard("green");
            }
        } 
        else 
        {
        rightBaseMotor.set(0);
        didShootPosition = false;
        // led.setBoard("red");
        }
        return didShootPosition;
    }


    private static Timer ampPosTime = new Timer();
    private boolean didAmpPosition = false;

    //#AMPPOSITION
    //This method will bring the manipulator to a position for it to score in the amp
    /*
     * @Param timeout           The time limit for the method
     * @Return didAmpPosition   Returns true if positioned successfully, returns false if not
     */
    public boolean ampPosition(double timeout) 
    {
        ampPosTime.reset();
        ampPosTime.start();

        if (ampPosTime.get() <= timeout) 
        {
            if (rightBaseEncoder.getPosition() <= Constants.ampPosition + 1) 
            {
            rightBaseMotor.set(-0.3);
            } 
            else 
            {
            rightBaseMotor.set(0);
            didAmpPosition = true;
            // led.setBoard("green");
            }
        } 
        else 
        {
        rightBaseMotor.set(0);
        didAmpPosition = false;
        // led.setBoard("red");
        }
        return didAmpPosition;
    }


        //#MANIPULATORDASHBOARD
        //This method updates the dashboard with all the data from the manipulator class
        public void manipulatorDashboard() 
        {
            //Tell if it is in manual or not
            SmartDashboard.putBoolean("Is Manual?", usingDigitalSensors);

            //Push the digital sensor data to the shuffleboard
            SmartDashboard.putBoolean("Beam Sensor", intakeSensor.get());
            SmartDashboard.putBoolean("Magnetic Sensor", frontSensor.get());

            //Push the encoder values to shuffleboard
            SmartDashboard.putNumber("Manipulator Base Encoder", rightBaseEncoder.getPosition());
        }



        public static boolean didIntake = false;
        private static Timer intakeTime = new Timer();

        //#INTAKE
        //This method will intake a note
        /*
         * @Param timeout   The time limit of the method
         */
        public void intake(double timeout) 
        {
            intakeTime.reset();
            intakeTime.start();

            if (intakeTime.get() <= timeout) 
            {
                if (!intakeSensor.get()) 
                {
                intakeMotor.set(0.4);
                } 
                else 
                {
                intakeMotor.set(0);
                didIntake = true;
                }
            } 
            else 
            {
            intakeMotor.set(0);
            didIntake = false;
            }
        }

        //#INTAKE
        //This method will manually intake a note
        public void intake() 
        {
            intakeMotor.set(0.4);
        }

        Timer revIntakeTime = new Timer();

        //#REVERSEINTAKE
        //This method runs the intake backwards for a short time to get the note out of the flywheels
        public void reverseIntake() 
        {
            revIntakeTime.reset();
            revIntakeTime.start();

            if (revIntakeTime.get() < 0.5) 
            {
                intakeMotor.set(-0.2);
            }
            else
            {
                intakeMotor.set(0);
            }
        }

        //#STOPINTAKE
        //This method halts the intake motors
        public void stopIntake() 
        {
            intakeMotor.set(0);
        }


        private Timer shootTime = new Timer();
        private static Timer shootTimeout = new Timer();

        //#SHOOTNOTE
        //This method will shoot a note
        /*
         * @Param timeout   The time limit of the method
         */
        public void shootNote(double timeout) 
        {

            shootTimeout.reset();
            shootTimeout.start();
            shootTime.reset();

            ampMotor.set(-0.6);
            
            if (shootTimeout.get() <= timeout) 
            {
            //If the beam sensor is active, the intake motor runs in reverse until the beam sensor is deactivated,
            //at which point the intake motor will stop and the amp motor will run for 1 second at full power to shoot
                if (intakeSensor.get()) 
                {
                    intakeMotor.set(0.3);
                } 
                else if (!intakeSensor.get()) 
                {
                    shootTime.start();
                } 
                if (!intakeSensor.get() && shootTime.get() >= 1) 
                {
                    intakeMotor.set(0);
                    ampMotor.set(0);
                    shootTime.stop();
                }
            }
        }

        //#SHOOTNOTE
        //This method will shoot a note manually
        public void shootNote() 
        {
            ampMotor.set(-0.5);
        }

        //#STOPSHOOT
        //This method stops the intake and amp motors
        public void stopShoot() 
        {
            ampMotor.set(0);
        }


        private Timer ampTime = new Timer();
        private static Timer ampTimeout = new Timer();

        //#AMPSCORE
        //This method will score a note in the amp
        /*
         * @Param timeout   The time limit of the method
         */
        public void ampScore(double timeout) 
        {

            ampTimeout.reset();
            ampTimeout.start();
            ampTime.reset();

            ampMotor.set(-0.3);

            if (ampTimeout.get() <= timeout) 
            {
            //If the beam sensor is active, the intake motor runs in reverse until the beam sensor is deactivated, at
            //which point the intake motor will stop and the amp motor will run for 1.5 seconds at 40% power to score
                if (intakeSensor.get()) 
                {
                intakeMotor.set(0.3);
                }
                else if (!intakeSensor.get()) 
                {
                ampTime.start();
                } 
                if (!intakeSensor.get() && ampTime.get() >= 1) 
                {
                ampMotor.set(0);
                intakeMotor.set(0);
                ampTime.stop(); 
                }
            }
        }

        //#AMPSCORE
        //This method will score in the amp manually
        public void ampScore() 
        {
            ampMotor.set(-0.3);
        }


        //#MOVEMANIPULATOR
        //This method will move the manipulator forward
        /*
         * @Param isNegative    Determines if the arm will move in a negative direction or not
         */
        public void moveManipulator(boolean isNegative) 
        {
            if (!isNegative) 
            {   
                rightBaseMotor.set(-0.3);
            } 
            else 
            {
                rightBaseMotor.set(0.3);
            }   
        }

        //#MOVEMANIPULATOR
        //This method will move the manipulator forward by a set time
        /*
         * @Param moveTime              The time for which the manipulator will move
         * @Param isNegative            Determines whether the robot will move in a positive or negative direction
         * @Return didMoveManipulator   Returns true when finished moving
         */
        private boolean didMoveManipulator = false;
        public boolean moveManipulator(double moveTime, boolean isNegative) 
        {
            Timer moveTimer = new Timer();
            moveTimer.reset();
            moveTimer.start();

            if (!isNegative) 
            {
                if (moveTimer.get() <= moveTime) 
                {
                rightBaseMotor.set(-0.2);
                } 
                else 
                {
                rightBaseMotor.set(0);
                didMoveManipulator = true;
                }
            } 
            else   
            {
                Timer moveTimer2 = new Timer();
                moveTimer2.reset();
                moveTimer2.start();

                if (moveTimer2.get() <= moveTime) 
                {
                rightBaseMotor.set(0.2);
                } else 
                {
                rightBaseMotor.set(0);
                didMoveManipulator = true;
                }
            }
            return didMoveManipulator;
        }


        //#RAMPUPMANIPULATOR
        //This method will increase the movement speed of the manipulator as the trigger is held more
        /*
         * @Param power The speed at which the manipulator will move (designed for use with XboxController triggers)
         */
        public void rampUpManipulator(double power) 
        {
            power *= 0.90;
            rightBaseMotor.set(power);
        }


        // #STOPMANIPULATOR
        //This method stops the manipulator motors
        public void stopManipulator() 
        {
            rightBaseMotor.set(0);
        }

        
            boolean setPos = false;
            double holdPos = 0;

        //#HOLDMANIPULATOR
        //This method will try to hold the manipulator in one spot
        public void holdManipulator() 
        {
            
            if (!setPos) 
            {
                holdPos = rightBaseEncoder.getPosition();
                setPos = true;
            }

            if (rightBaseEncoder.getPosition() > holdPos + 1.5)
            {
                rightBaseMotor.set(-0.2);
            }
            if (rightBaseEncoder.getPosition() < holdPos - 1.5)
            {
                rightBaseMotor.set(0.2);
            }
            if (rightBaseEncoder.getPosition() < holdPos + 1.5 && rightBaseEncoder.getPosition() > holdPos - 1.5) 
            {
                rightBaseMotor.set(0);
            }
        }


        //#CONTROLMANIPULATOR
        //This method will add keybinds for all the control methods in the manipulator class
        public void controlManipulator() 
        {
            if (IO.dController.getStartButtonPressed()) manualControl();
            
            if (!usingDigitalSensors) 
            {
                if (IO.dController.getRightTriggerAxis() > 0.2 && backSensor.get()) rampUpManipulator(-IO.dController.getRightTriggerAxis());
                if (IO.dController.getLeftTriggerAxis() > 0.2 && frontSensor.get()) rampUpManipulator(IO.dController.getLeftTriggerAxis());
                if ( !IO.oController.getYButton() && !IO.oController.getXButton() && (( IO.oController.getLeftTriggerAxis() < 0.2 && IO.dController.getRightTriggerAxis() < 0.2 ) || ( IO.dController.getLeftTriggerAxis() > 0.2 && IO.dController.getRightTriggerAxis() > 0.2 ))) stopManipulator();
                if (IO.oController.getAButton()) holdManipulator();
                if (IO.dController.getRightBumper()) shootNote();
                if (IO.dController.getLeftBumper()) intake();
                if (IO.dController.getAButtonPressed()) reverseIntake();
                if (IO.dController.getBButton()) ampScore();
                if (!IO.dController.getRightBumper()) stopShoot();
                if (IO.oController.getYButtonPressed()) intakePosition(5);
                if (IO.oController.getXButton()) shootPosition();
                if (!IO.dController.getLeftBumper() && !IO.dController.getAButton()) stopIntake();
                if (IO.oController.getBButton()) 
                {
                    if (ampPosition(5)) ampScore(4);
                }
            } 
            else 
            {
                if (IO.dController.getRightTriggerAxis() > 0.2) rampUpManipulator(-IO.dController.getRightTriggerAxis());
                if (IO.dController.getLeftTriggerAxis() > 0.2 && frontSensor.get()) rampUpManipulator(IO.dController.getLeftTriggerAxis());
                if ( ( IO.dController.getLeftTriggerAxis() < 0.2 && IO.dController.getRightTriggerAxis() < 0.2 ) || ( IO.dController.getLeftTriggerAxis() > 0.2 && IO.dController.getRightTriggerAxis() > 0.2 )) stopManipulator();
                if (IO.oController.getAButton()) holdManipulator();
                if (IO.dController.getRightBumper()) shootNote();
                if (IO.dController.getLeftBumper()) intake();
                if (IO.dController.getAButtonPressed()) reverseIntake();
                if (IO.dController.getBButton()) ampScore();
                if (!IO.dController.getRightBumper()) stopShoot();
                if (!IO.dController.getLeftBumper() && !IO.dController.getAButton()) stopIntake();
            }
        }



        //#AUTOMANIPULATOR
        //This method will do all of the actions for our manipulator during auto
        /*
         * @Param doesIntake        If true, then the manipulator will go to the intake position and intake a note
         * @Param doesAim           If true, then the manipulator will go to the shooting position
         * @Param doesShoot         If true, then the manipulator will shoot a note
         * @Param doesAmpAim        If true, then the manipulator will go to the amp position
         * @Param doesAmp           If true, then the manipulator will score in the amp
         */
        public void autoManipulator(boolean doesIntake, boolean doesAim, boolean doesShoot, boolean doesAmpAim, boolean doesAmp) 
        {
            if (doesIntake) 
            {
                if (intakePosition(5)) intake(3);
            }
            if (doesAim) shootPosition(5);
            if (doesShoot) shootNote(3);
            if (doesAmpAim) ampPosition(5);
            if (doesAmp) ampScore(4);
        }
}

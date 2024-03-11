package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class MoveForwardInches extends Command{
    private final DriveTrain drivetrain;
    private final Manipulator manipulator;
    private final double speed;
    private final double distance;
    private double travelledDistance = 0.0;

    public MoveForwardInches(DriveTrain dT, Manipulator M, double speed, double distance){
        drivetrain = dT;
        manipulator = M;
        this.speed = speed;
        this.distance = Math.abs(distance);
    }

    @Override
    public void initialize(){
        drivetrain.leftEncoder.setPosition(0.0);
        drivetrain.rightEncoder.setPosition(0.0);
        manipulator.runIntake(false, true);
    }

    @Override
    public void execute(){
        travelledDistance += Math.abs(drivetrain.rightEncoder.getPosition() / 8.45 * 18);

        //Drive forward until the distance has been travelled
        if (travelledDistance < distance) drivetrain.drive(speed, 0);
    }

    @Override
    public boolean isFinished(){
        return travelledDistance >= distance;
    }
    @Override
    public void end(boolean interrupted){
        manipulator.runIntake(false, false);
        drivetrain.drive(0, 0);
    }
}
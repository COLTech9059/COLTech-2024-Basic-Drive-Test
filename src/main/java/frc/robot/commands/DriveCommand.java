package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.IO;

public class DriveCommand extends Command
{
    
    private final DriveTrain drivetrain;

    private DoubleSupplier forward;
    private DoubleSupplier turn;
    
    public DriveCommand(DriveTrain dT, DoubleSupplier f, DoubleSupplier t)
    {
        drivetrain = dT;
        forward = f;
        turn = t;
        addRequirements(drivetrain);
    }

    @Override
    public void execute()
    {
        drivetrain.HamsterDrive.arcadeDrive(forward.getAsDouble(), turn.getAsDouble(), true);
    }

}


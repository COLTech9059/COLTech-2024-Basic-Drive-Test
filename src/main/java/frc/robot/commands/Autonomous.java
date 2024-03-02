package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(DriveTrain dT, LimeLight LL){
        addCommands(new LimeLightCommand(LL, dT));
    }
}

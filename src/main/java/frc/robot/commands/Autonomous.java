package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(DriveTrain dT, LimeLight LL, Manipulator M){
        //Add more commands to this list to increase the functionality of Autonomous.
        addCommands(
            new SpeakerScore(M, true, 1.7, 2.5),
            new MoveForwardInches(dT, M, .35, 36.0),
            new LimeLightCommand(LL, dT),
            new SpeakerScore(M, false, .5, 1)
            );
    }
}

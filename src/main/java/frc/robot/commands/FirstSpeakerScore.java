package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Manipulator;

public class FirstSpeakerScore extends Command{
    private final Manipulator m_Manipulator;
    private final Timer moveTime = new Timer();
    private boolean Completed = false;
    public FirstSpeakerScore(Manipulator manip){
        m_Manipulator = manip;
        addRequirements(m_Manipulator);
    }
    @Override
    public void initialize(){
        m_Manipulator.moveArm(-.3);
        m_Manipulator.shootNote(true, false);
        moveTime.start();
    }
    @Override
    public void execute(){
        if (moveTime.get() >= 1.7)
        {
            m_Manipulator.runIntake(false, true);
        }
        if (moveTime.get() >= 2.5)
        {
            m_Manipulator.runIntake(false, false);
            m_Manipulator.shootNote(false, false);
            Completed = true;
        }
    }
    @Override
    public boolean isFinished(){
        return Completed;
    }
    @Override
    public void end(boolean interrupted){
        Completed = false;
        m_Manipulator.shootNote(false, false);
        m_Manipulator.runIntake(false, false);

        moveTime.stop();
        moveTime.reset();
    }
}

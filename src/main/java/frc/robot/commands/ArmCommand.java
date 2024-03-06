package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class ArmCommand extends Command {

    private final Manipulator m_Manipulator;
    private final DoubleSupplier ArmPower;
    private final BooleanSupplier shootEnabled;
    private final BooleanSupplier intakeActive;
    private final BooleanSupplier canReverseIntake;

    public ArmCommand(Manipulator sentManip, DoubleSupplier armPower, BooleanSupplier shootActive, BooleanSupplier intakeActive, BooleanSupplier canReverseIntake){
        //Initialize DoubleSuppliers and the Manipulator.
        m_Manipulator = sentManip;
        //For Evan: we can use a singular armPower variable to make things simpler
        //and also be able to control arm speed with the other trigger.
        ArmPower = armPower;
        shootEnabled = shootActive;
        this.intakeActive = intakeActive;
        this.canReverseIntake = canReverseIntake;

        addRequirements(m_Manipulator);
    }

    @Override
    public void execute(){
        //Move arm based on power
        m_Manipulator.moveArm(ArmPower.getAsDouble());
        m_Manipulator.shootNote(shootEnabled.getAsBoolean());
        m_Manipulator.runIntake(canReverseIntake.getAsBoolean(), intakeActive.getAsBoolean());
        // m_Manipulator.
    }
}

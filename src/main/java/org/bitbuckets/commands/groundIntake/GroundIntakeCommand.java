package org.bitbuckets.commands.groundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;

public class GroundIntakeCommand extends Command {

    final GroundIntakeSubsystem groundIntakeSubsystem;
    final OperatorInput operatorInput;


    public GroundIntakeCommand(GroundIntakeSubsystem groundIntakeSubsystem, OperatorInput operatorInput) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.operatorInput = operatorInput;
    }

    @Override
    public void execute() {
        groundIntakeSubsystem.setToVoltage(6); //TODO tune voltage for intake
    }

    @Override
    public void end(boolean interrupted) {
        groundIntakeSubsystem.setToVoltage(0);
    }

}
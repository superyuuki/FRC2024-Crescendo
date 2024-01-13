package org.bitbuckets.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.shooter.ShooterSubsystem;

public class SetSpeakerShootingAngleCommand extends Command {
        private final ShooterSubsystem shooterSubsystem;

        public SetSpeakerShootingAngleCommand(ShooterSubsystem shooterSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
        }

        @Override
        public void initialize() {

        }

        // this angle needs to be tuned for speaker (60 is only a placeholder)
        @Override
        public void execute() {
            shooterSubsystem.moveToAngle(60);
        }

        @Override
        public void end(boolean interrupted) {
            shooterSubsystem.moveToAngle(0);
        }
    }



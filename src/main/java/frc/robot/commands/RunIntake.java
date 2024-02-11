package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake {

    public class PivotIntake extends Command {

        private IntakeSubsystem intakeSubsystem;
        private final Supplier<Boolean> rightBumper, leftBumper;
    
        public PivotIntake(IntakeSubsystem intake, Supplier<Boolean> rightBumper, Supplier<Boolean> leftBumper){
            this.intakeSubsystem = intake;
            this.rightBumper = rightBumper;
            this.leftBumper = leftBumper;
            addRequirements(intake);
        }
    
        @Override
        public void initialize() {
        }
    
        @Override
        public void execute() {
            if(rightBumper.get()){
                intakeSubsystem.intakeForward();
            }
            else {
                intakeSubsystem.intakeStop();
            }
    
            if(leftBumper.get()){
                intakeSubsystem.intakeReverse();
            }
            else {
                intakeSubsystem.intakeStop();
            }

        }
    
        @Override
        public void end(boolean interrupted) {
        }
    
        @Override
        public boolean isFinished(){
            return false;
        }
    }
}
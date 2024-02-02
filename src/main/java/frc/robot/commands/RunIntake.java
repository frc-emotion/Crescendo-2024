package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake {
    public class PivotIntake extends Command {

        private Intake intake;
        private final Supplier<Boolean> rightBumper, leftBumper;
    
        public PivotIntake(Intake intake, Supplier<Boolean> rightBumper, Supplier<Boolean> leftBumper){
            this.intake = intake;
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
                intake.intakeForward();
            }
            else {
                intake.intakeStop();
            }
    
            if(leftBumper.get()){
                intake.intakeReverse();
            }
            else {
                intake.intakeStop();
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

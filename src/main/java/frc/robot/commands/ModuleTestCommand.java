package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Module;

public class ModuleTestCommand extends CommandBase {
    private Module module;
    public ModuleTestCommand(Module module){
        this.module = module;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turn Angle", 0);
        SmartDashboard.putNumber("Move %", 0);
    }

    @Override
    public void execute() {
        module.periodic();
        module.set(
            SmartDashboard.getNumber("Move %", 0),
            SmartDashboard.getNumber("Turn Angle", 0) * Constants.TAU/360
            );
    }

    @Override
    public void end(boolean interrupted) {
        module.set(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

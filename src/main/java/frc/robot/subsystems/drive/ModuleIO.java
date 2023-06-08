package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double movePosition; // Rad
        public double moveVelocity; // Rad/sec
        public double moveStatorCurrent; // Amps
        public double moveSupplyCurrent; // Amps

        public double turnPositionRotor; // Rad
        public double turnPositionEncoder;
        public double turnVelocity; // Rad/sec
        public double turnStatorCurrent; // Amps
        public double turnSupplyCurrent; // Amps
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setTurn(double angle);

    public void setMove(double speed);
    
}

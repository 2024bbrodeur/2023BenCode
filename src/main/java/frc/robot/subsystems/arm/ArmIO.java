package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOInputs {
        public double extendPosition; // M
        public double extendVelocity; // M/sec
        public double extendStatorCurrent; // Amps
        public double extendSupplyCurrent; // Amps

        public double pivotPositionEncoder; // Rad

        public double pivot1PositionRotor; // Rad
        public double pivot1Velocity; // Rad/sec
        public double pivot1StatorCurrent; // Amps
        public double pivot1SupplyCurrent; // Amps

        public double pivot2PositionRotor; // Rad
        public double pivot2Velocity; // Rad/sec
        public double pivot2StatorCurrent; // Amps
        public double pivot2SupplyCurrent; // Amps
    }

    public void updateInputs(ArmIOInputs inputs);

    public void setExtendPosition(double position);
    public void setExtendVelocity(double velocity);

    public void setPivotPosition(double angle);
    public void setPivotVelocity(double velocity);

}

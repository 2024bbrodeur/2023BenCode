package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class Module {
    
    public int index;

    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io, int index) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module"+index, inputs);
    }

    public void set(SwerveModuleState state) {
        set(state.speedMetersPerSecond / Constants.Drive.MAX_MOVE_VELOCITY_MPS, NRUnits.constrainRad(state.angle.getRadians()));
    }

    /*
     * move: [-1, 1]
     * angle: [-Tau/2, Tau/2]
     */
    public void set(double move, double angle) {

    }

}
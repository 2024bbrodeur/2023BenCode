package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.util.SwerveState;

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

        SmartDashboard.putNumber("Angle", inputs.turnPositionRotor * 360/Constants.TAU);
    }

    public void set(SwerveModuleState state) {
        set(state.speedMetersPerSecond / Constants.Drive.MAX_MOVE_VELOCITY_MPS, NRUnits.constrainRad(state.angle.getRadians()));
    }

    public void set(SwerveState state) {
        set(state.move, state.turn);
    }

    double lastAngle = 0;
    double mult = 1;

    /*
     * move: [-1, 1]
     * angle: [-Tau/2, Tau/2]
     */
    public void set(double move, double angle) {
        angle = NRUnits.constrainRad(angle);
        double diff = Math.abs(angle - inputs.turnPositionRotor);
        if(diff < Constants.TAU/4 || diff > 3*Constants.TAU/4) {
            io.setMove(move);
            io.setTurn(angle);
        } else {
            io.setMove(-move);
            io.setTurn(NRUnits.constrainRad(angle + Constants.TAU/2));
        }
    }

}
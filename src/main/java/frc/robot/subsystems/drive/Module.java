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
        // if(angle != lastAngle) {
        //     double diff = Math.abs(angle - inputs.turnPositionRotor);
        //     SmartDashboard.putNumber("Diff", diff);
            
        //     if(diff < Constants.TAU/4 || diff > Constants.TAU*3/4) {
        //         io.setTurn(angle);
        //         mult = 1;
        //     } else {
        //         io.setTurn(NRUnits.constrainRad(angle + Constants.TAU/2));
        //         mult = -1;
        //     }
        // }
        // io.setMove(move * mult);
        // lastAngle = angle;
    }

}
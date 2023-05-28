package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.math.NRUnits;

public class SwerveState {

    public double move;
    public double turn;

    public SwerveState(double move, double turn) {
        this.move = move;
        this.turn = turn;
    }

    public SwerveState(Translation2d state) {
        move = state.getDistance(new Translation2d(0, 0));
        turn = NRUnits.constrainRad(state.getAngle().getRadians());
    }

    public static SwerveState fromTranslation2d(Translation2d state) {
        return new SwerveState(state.getDistance(new Translation2d(0, 0)), NRUnits.constrainRad(state.getAngle().getRadians()));
    }

    public static SwerveState[] normalize(SwerveState[] states) {
        double max = 0;
        for(SwerveState state : states) {
            max = state.move > max ? state.move : max;
        }

        if(max > 1) {
            for(SwerveState state : states) {
                state.move /= max;
            }
        }

        return states;
    }
    
}

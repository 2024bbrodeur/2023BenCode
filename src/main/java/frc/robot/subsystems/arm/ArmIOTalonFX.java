package frc.robot.subsystems.arm;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class ArmIOTalonFX implements ArmIO {
    
    private TalonFX extend;
    private TalonFXConfiguration extendConfig;
    private TalonFXConfigurator extendConfigurator;

    private TalonFX pivot1;
    private TalonFXConfigurator pivot1Configurator;

    private TalonFX pivot2;
    private TalonFXConfigurator pivot2Configurator;

    private TalonFXConfiguration pivotConfig;

    private CANcoder pivotEncoder;
    private CANcoderConfiguration pivotEncoderConfig;
    private CANcoderConfigurator pivotEncoderConfigurator;

    public ArmIOTalonFX() {
        extend = new TalonFX(Constants.Arm.EXTEND_PORT, "drivet");
        extendConfig = new TalonFXConfiguration();
        extendConfigurator = extend.getConfigurator();

        pivot1 = new TalonFX(Constants.Arm.PIVOT1_PORT, "drivet");
        pivot1Configurator = pivot1.getConfigurator();

        pivot2 = new TalonFX(Constants.Arm.PIVOT2_PORT, "drivet");
        pivot2Configurator = pivot2.getConfigurator();

        pivotConfig = new TalonFXConfiguration();

        pivotEncoder = new CANcoder(Constants.Arm.ENCODER_PORT, "drivet");
        pivotEncoderConfig = new CANcoderConfiguration();
        pivotEncoderConfigurator = pivotEncoder.getConfigurator();

        config();
    }

    public void setExtendPosition(double position) {

    }

    public void setExtendVelocity(double velocity) {

    }

    public void setPivotPosition(double angle) {

    }

    public void setPivotVelocity(double velocity) {

    }

    private void config() {
        extendConfig.Audio.BeepOnBoot = true;
        extendConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        extendConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        extendConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        extendConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        extendConfig.FutureProofConfigs = true;
        extendConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.EXTEND_CRUISE_VELOCITY;
        extendConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.EXTEND_ACCELERATION;
        extendConfig.MotionMagic.MotionMagicJerk = Constants.Arm.EXTEND_JERK;
        extendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extendConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extendConfig.Slot0.kS = Constants.Arm.EXTEND_KS;
        extendConfig.Slot0.kV = Constants.Arm.EXTEND_KV;
        extendConfig.Slot0.kP = Constants.Arm.EXTEND_KP;
        extendConfig.Slot0.kI = Constants.Arm.EXTEND_KI;
        extendConfig.Slot0.kD = Constants.Arm.EXTEND_KD;
        extendConfig.Voltage.PeakForwardVoltage = 12;
        extendConfig.Voltage.PeakReverseVoltage = 12;
        extendConfigurator.apply(extendConfig);

        pivotConfig.Audio.BeepOnBoot = true;
        pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
        pivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.FutureProofConfigs = true;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.PIVOT_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.PIVOT_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Arm.PIVOT_JERK;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Slot0.kS = Constants.Arm.PIVOT_KS;
        pivotConfig.Slot0.kV = Constants.Arm.PIVOT_KV;
        pivotConfig.Slot0.kP = Constants.Arm.PIVOT_KP;
        pivotConfig.Slot0.kI = Constants.Arm.PIVOT_KI;
        pivotConfig.Slot0.kD = Constants.Arm.PIVOT_KD;
        pivotConfig.Voltage.PeakForwardVoltage = 12;
        pivotConfig.Voltage.PeakReverseVoltage = 12;
        pivot1Configurator.apply(pivotConfig);
        pivot2Configurator.apply(pivotConfig);
        Follower pivotFollower = new Follower(Constants.Arm.PIVOT1_PORT, true);
        pivot2.setControl(pivotFollower);

        pivotEncoderConfig.FutureProofConfigs = true;
        pivotEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        pivotEncoderConfig.MagnetSensor.MagnetOffset = Constants.Arm.ENCODER_OFFSET;
        pivotEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoderConfigurator.apply(pivotEncoderConfig);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.extendPosition = extend.getPosition().getValue();
        inputs.extendVelocity = extend.getVelocity().getValue();
        inputs.extendStatorCurrent = extend.getStatorCurrent().getValue();
        inputs.extendSupplyCurrent = extend.getSupplyCurrent().getValue();

        inputs.pivot1PositionRotor = pivot1.getPosition().getValue();
        inputs.pivot1Velocity = pivot1.getVelocity().getValue();
        inputs.pivot1StatorCurrent = pivot1.getStatorCurrent().getValue();
        inputs.pivot1SupplyCurrent = pivot1.getSupplyCurrent().getValue();

        inputs.pivot2PositionRotor = pivot2.getPosition().getValue();
        inputs.pivot2Velocity = pivot2.getVelocity().getValue();
        inputs.pivot2StatorCurrent = pivot2.getStatorCurrent().getValue();
        inputs.pivot2SupplyCurrent = pivot2.getSupplyCurrent().getValue();

        inputs.pivotPositionEncoder = pivotEncoder.getAbsolutePosition().getValue();
    }

}

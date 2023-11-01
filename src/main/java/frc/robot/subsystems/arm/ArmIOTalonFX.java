package frc.robot.subsystems.arm;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
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
    private MotionMagicDutyCycle extendPositionControl;
    private VelocityDutyCycle extendVelocityControl;

    private TalonFX pivot1;
    private TalonFXConfigurator pivot1Configurator;

    private TalonFX pivot2;
    private TalonFXConfigurator pivot2Configurator;

    private TalonFXConfiguration pivotConfig;
    private MotionMagicDutyCycle pivotPositionControl;
    private VelocityDutyCycle pivotVelocityControl;

    private CANcoder pivotEncoder;
    private CANcoderConfiguration pivotEncoderConfig;
    private CANcoderConfigurator pivotEncoderConfigurator;

    public ArmIOTalonFX() {
        extend = new TalonFX(Constants.Arm.Extend.PORT, "drivet");
        extendConfig = new TalonFXConfiguration();
        extendConfigurator = extend.getConfigurator();
        extendPositionControl = new MotionMagicDutyCycle(0, true, 0, 0, false);
        extendVelocityControl = new VelocityDutyCycle(0, true, 0, 0, false);

        pivot1 = new TalonFX(Constants.Arm.Pivot.PORT1, "drivet");
        pivot1Configurator = pivot1.getConfigurator();

        pivot2 = new TalonFX(Constants.Arm.Pivot.PORT2, "drivet");
        pivot2Configurator = pivot2.getConfigurator();

        pivotConfig = new TalonFXConfiguration();
        pivotPositionControl = new MotionMagicDutyCycle(0, true, 0, 0, false);
        pivotVelocityControl = new VelocityDutyCycle(0, true, 0, 0, false);

        pivotEncoder = new CANcoder(Constants.Arm.Pivot.ENCODER_PORT, "drivet");
        pivotEncoderConfig = new CANcoderConfiguration();
        pivotEncoderConfigurator = pivotEncoder.getConfigurator();

        config();
    }

    // position: m
    public void setExtendPosition(double position) {
        extendPositionControl.Position = position * Constants.Arm.Extend.NU_PER_METERS;
        extend.setControl(extendPositionControl);
    }

    // velocity: m/s
    public void setExtendVelocity(double velocity) {
        extendVelocityControl.Velocity = velocity * Constants.Arm.Extend.NU_PER_METERS;
        extend.setControl(extendVelocityControl);
    }

    // angle: rad
    public void setPivotPosition(double angle) {
        pivotPositionControl.Position = angle * Constants.Arm.Pivot.NU_PER_RADIAN;
        pivot1.setControl(pivotPositionControl);
    }

    // velocity: rad/s
    public void setPivotVelocity(double velocity) {
        pivotVelocityControl.Velocity = velocity * Constants.Arm.Pivot.NU_PER_RADIAN;
        pivot1.setControl(pivotVelocityControl);
    }

    private void config() {
        extendConfig.Audio.BeepOnBoot = true;
        extendConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        extendConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        extendConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        extendConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        extendConfig.FutureProofConfigs = true;
        extendConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.Extend.CRUISE_VELOCITY;
        extendConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.Extend.ACCELERATION;
        extendConfig.MotionMagic.MotionMagicJerk = Constants.Arm.Extend.JERK;
        extendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extendConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extendConfig.Slot0.kS = Constants.Arm.Extend.KS;
        extendConfig.Slot0.kV = Constants.Arm.Extend.KV;
        extendConfig.Slot0.kP = Constants.Arm.Extend.KP;
        extendConfig.Slot0.kI = Constants.Arm.Extend.KI;
        extendConfig.Slot0.kD = Constants.Arm.Extend.KD;
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
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.Pivot.CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.Pivot.ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Arm.Pivot.JERK;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Slot0.kS = Constants.Arm.Pivot.KS;
        pivotConfig.Slot0.kV = Constants.Arm.Pivot.KV;
        pivotConfig.Slot0.kP = Constants.Arm.Pivot.KP;
        pivotConfig.Slot0.kI = Constants.Arm.Pivot.KI;
        pivotConfig.Slot0.kD = Constants.Arm.Pivot.KD;
        pivotConfig.Voltage.PeakForwardVoltage = 12;
        pivotConfig.Voltage.PeakReverseVoltage = 12;
        pivot1Configurator.apply(pivotConfig);
        pivot2Configurator.apply(pivotConfig);
        Follower pivotFollower = new Follower(Constants.Arm.Pivot.PORT1, true);
        pivot2.setControl(pivotFollower);

        pivotEncoderConfig.FutureProofConfigs = true;
        pivotEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        pivotEncoderConfig.MagnetSensor.MagnetOffset = Constants.Arm.Pivot.ENCODER_OFFSET;
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

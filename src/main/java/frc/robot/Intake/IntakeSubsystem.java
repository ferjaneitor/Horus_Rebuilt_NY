package frc.robot.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Drive.TunerConstants;

/**
 * Subsistema del roller del intake.
 *
 * Motor: Kraken X60 / TalonFX (ID 14, CANivore)
 * Control: DutyCycleOut (ciclo de trabajo directo).
 *   Negativo = giro CW = absorber pelotas.
 *   Positivo = giro CCW = expulsar pelotas.
 *
 * TODO (futuro): migrar a TorqueCurrentFOC + auto-desatasco cuando
 * se confirme la licencia Phoenix Pro y se calibren los limites.
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Intake/CommandedDuty    — ciclo de trabajo comandado [-1.0, 1.0]
 * Intake/StatorAmps       — corriente de stator medida [A]
 * Intake/RollerRPS        — velocidad angular del roller [RPS]
 * Intake/RollerRotations  — posicion acumulada del roller [rotaciones]
 * Intake/IsRunning        — true si el roller esta activo
 * </pre>
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;

    // Signals cacheadas
    @SuppressWarnings("rawtypes")
    private final StatusSignal statorCurrentSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal rollerPositionSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal rollerVelocitySignal;

    // Control requests reutilizables
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0.0);
    private final NeutralOut   neutralRequest = new NeutralOut();

    // Estado para telemetria
    private double  commandedDuty = 0.0;
    private boolean isRunning     = false;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(IntakeConstants.INTAKE_NEUTRAL_MODE)
            .withInverted(
                IntakeConstants.INTAKE_INVERTED
                    ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                    : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive
            );

        // Limite de corriente de stator — proteccion hardware
        config.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);

        rollerMotor.getConfigurator().apply(config);

        // Cachear signals
        statorCurrentSignal  = rollerMotor.getStatorCurrent();
        rollerPositionSignal = rollerMotor.getPosition();
        rollerVelocitySignal = rollerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            IntakeConstants.INTAKE_SIGNAL_HZ,
            statorCurrentSignal, rollerPositionSignal, rollerVelocitySignal
        );
        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(statorCurrentSignal, rollerPositionSignal, rollerVelocitySignal);

        SmartDashboard.putNumber ("Intake/CommandedDuty",   commandedDuty);
        SmartDashboard.putNumber ("Intake/StatorAmps",      statorCurrentSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Intake/RollerRPS",       rollerVelocitySignal.getValueAsDouble());
        SmartDashboard.putNumber ("Intake/RollerRotations", rollerPositionSignal.getValueAsDouble());
        SmartDashboard.putBoolean("Intake/IsRunning",       isRunning);
    }

    // =========================================================================
    // API publica
    // =========================================================================

    /**
     * Corre el roller con el ciclo de trabajo especificado.
     * Negativo = absorber pelotas. Positivo = expulsar.
     * Se clampea al maximo configurado.
     *
     * @param duty Ciclo de trabajo [-ROLLER_MANUAL_MAX_DUTY, +ROLLER_MANUAL_MAX_DUTY].
     */
    public void runIntake(double duty) {
        commandedDuty = Math.max(-IntakeConstants.ROLLER_MANUAL_MAX_DUTY,
                        Math.min( IntakeConstants.ROLLER_MANUAL_MAX_DUTY, duty));
        isRunning = commandedDuty != 0.0;
        rollerMotor.setControl(rollerRequest.withOutput(commandedDuty));
    }

    /**
     * Corre el roller en modo automatico con el ciclo de trabajo configurado
     * ({@code INTAKE_ROLLER_DUTY_CYCLE} — negativo = absorber).
     */
    public void runIntake() {
        commandedDuty = IntakeConstants.INTAKE_ROLLER_DUTY_CYCLE;
        isRunning     = true;
        rollerMotor.setControl(rollerRequest.withOutput(commandedDuty));
    }

    /**
     * Detiene el roller (NeutralOut — coast segun configuracion).
     */
    public void stopIntake() {
        commandedDuty = 0.0;
        isRunning     = false;
        rollerMotor.setControl(neutralRequest);
    }

    /** {@code true} si el roller esta actualmente girando. */
    public boolean isRunning() {
        return isRunning;
    }
}

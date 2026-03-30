package frc.robot.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsistema del pivot del intake.
 *
 * Motor: SparkMax + NEO v1.1 (ID 19)
 *
 * El PID del SparkMax corre continuamente en periodic().
 * Los comandos unicamente modifican el campo currentSetpoint.
 * Esto evita que multiples comandos peleen entre si por el control del motor.
 *
 * <h2>Compensacion de gravedad</h2>
 * <p>El brazo del intake es pesado — la gravedad lo jala hacia la posicion desplegada.
 * Se aplica un feedforward positivo constante ({@link IntakeConstants#PIVOT_GRAVITY_FF_VOLTS})
 * via {@code ArbFFUnits.kVoltage} en {@code setReference()} para contrarrestar el peso
 * en todo momento cuando el PID esta activo.</p>
 *
 * Posiciones:
 *   - Retractado (arriba): 0.0 rotaciones del mecanismo
 *   - Desplegado (abajo):  PIVOT_DEPLOYED_POSITION_ROTATIONS (TODO: calibrar)
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax pivotMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    // Setpoint actual en rotaciones del mecanismo (post gear ratio).
    // Se inicializa en 0 asumiendo que el robot arranca con el pivot retractado.
    private double currentSetpoint = IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS;

    // Control manual — cuando esta activo, periodic() aplica manualOutput en lugar del PID.
    // El PID se restaura automaticamente al llamar disableManualOverride().
    private boolean manualOverride = false;
    private double  manualOutput   = 0.0;

    public IntakePivotSubsystem() {
        pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
        encoder = pivotMotor.getEncoder();
        closedLoopController = pivotMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(IntakeConstants.INTAKE_PIVOT_INVERTED)
            .idleMode(IntakeConstants.INTAKE_PIVOT_IDLE_MODE)
            .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT);

        // Factor de conversion: rotaciones del motor -> rotaciones del mecanismo.
        // Se divide entre el gear ratio para que el encoder reporte en unidades del mecanismo.
        // TODO: Actualizar PIVOT_GEAR_RATIO en Constants cuando se conozca el valor real.
        config.encoder
            .positionConversionFactor(1.0 / IntakeConstants.PIVOT_GEAR_RATIO);

        // Configuracion del PID onboard del SparkMax (slot 0).
        // TODO: Tunear PIVOT_P, PIVOT_I, PIVOT_D en el robot fisico.
        config.closedLoop
            .pid(
                IntakeConstants.PIVOT_P,
                IntakeConstants.PIVOT_I,
                IntakeConstants.PIVOT_D
            )
            .outputRange(IntakeConstants.PIVOT_MIN_OUTPUT, IntakeConstants.PIVOT_MAX_OUTPUT);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // El robot arranca con el pivot en posicion retractada — inicializar el encoder en ese valor.
        // Esto garantiza que el PID no intente moverse al arrancar.
        encoder.setPosition(IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS);
    }

    /**
     * Se llama cada 20 ms.
     *
     * <ul>
     *   <li>Modo normal: aplica continuamente el setpoint al PID del SparkMax.</li>
     *   <li>Modo manual: aplica una salida de ciclo de trabajo fija (sin PID).
     *       Se activa con {@link #setManualOutput} y se desactiva con
     *       {@link #disableManualOverride}.</li>
     * </ul>
     */
    @Override
    public void periodic() {
        if (manualOverride) {
            // Control manual: salida directa de ciclo de trabajo, PID inactivo
            pivotMotor.set(manualOutput);
        } else {
            // Control PID: el SparkMax mantiene la posicion objetivo con feedforward de gravedad.
            // ArbFFUnits.kVoltage aplica un voltaje positivo constante hacia arriba
            // para contrarrestar el peso del brazo independientemente del angulo.
            closedLoopController.setSetpoint(
                currentSetpoint,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                IntakeConstants.PIVOT_GRAVITY_FF_VOLTS,
                ArbFFUnits.kVoltage
            );
        }

        // Telemetria para debugging y tuning
        SmartDashboard.putNumber ("IntakePivot/Position",       getPosition());
        SmartDashboard.putNumber ("IntakePivot/Setpoint",       currentSetpoint);
        SmartDashboard.putBoolean("IntakePivot/AtSetpoint",     atSetpoint());
        SmartDashboard.putBoolean("IntakePivot/ManualOverride", manualOverride);
    }

    /**
     * Cambia el setpoint objetivo del pivot.
     * El PID en periodic() se encarga de mover el motor hacia esta posicion.
     *
     * @param rotations Posicion objetivo en rotaciones del mecanismo (post gear ratio).
     */
    public void setSetpoint(double rotations) {
        currentSetpoint = rotations;
    }

    /**
     * Retorna la posicion actual del mecanismo en rotaciones (post gear ratio).
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Retorna true si el pivot esta dentro de la tolerancia del setpoint actual.
     */
    public boolean atSetpoint() {
        return Math.abs(getPosition() - currentSetpoint) < IntakeConstants.PIVOT_POSITION_TOLERANCE_ROTATIONS;
    }

    /**
     * Retorna el setpoint actual (util para telemetria o condiciones en comandos).
     */
    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    // =========================================================================
    // Control manual (override del PID)
    // =========================================================================

    /**
     * Activa el modo de control manual y aplica la salida indicada.
     *
     * <p>Mientras el override este activo, {@link #periodic()} llama
     * {@code pivotMotor.set(manualOutput)} en lugar de aplicar el PID.
     * El PID retoma el control cuando se llama {@link #disableManualOverride()}.</p>
     *
     * <p>La salida se clampea a [-PIVOT_MANUAL_MAX_OUTPUT, +PIVOT_MANUAL_MAX_OUTPUT]
     * para evitar movimientos bruscos durante pruebas.</p>
     *
     * @param percent Salida en [-1.0, 1.0].
     */
    public void setManualOutput(double percent) {
        manualOutput   = Math.max(-IntakeConstants.PIVOT_MANUAL_MAX_OUTPUT,
                         Math.min( IntakeConstants.PIVOT_MANUAL_MAX_OUTPUT, percent));
        manualOverride = true;
    }

    /**
     * Desactiva el override manual y restaura el control PID.
     *
     * <p>El pivot volvera a mantener {@code currentSetpoint}. Si se desea
     * mover el pivot a una nueva posicion, llamar {@link #setSetpoint(double)}
     * despues de este metodo.</p>
     */
    public void disableManualOverride() {
        currentSetpoint = encoder.getPosition();  // Mantener posicion actual — no correr al setpoint previo
        manualOutput    = 0.0;
        manualOverride  = false;
    }
}

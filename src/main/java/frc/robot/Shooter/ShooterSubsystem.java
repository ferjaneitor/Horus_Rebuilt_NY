package frc.robot.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Drive.TunerConstants;

/**
 * Subsistema del shooter — controla flywheel, hood (angulo de disparo) e indexer.
 *
 * <h2>Motores</h2>
 * <ul>
 *   <li><b>Flywheel izquierdo (ID 16)</b> y <b>derecho (ID 17)</b>:
 *       Kraken X60 (TalonFX) — VelocityVoltage + FOC. kS+kV+kA para FF optimo.</li>
 *   <li><b>Hood / angulo (ID 20)</b>:
 *       Kraken X60 (TalonFX) — MotionMagicVoltage + FOC. kG Arm_Cosine para gravedad.</li>
 *   <li><b>Indexer (ID 22)</b>:
 *       Kraken X60 (TalonFX) — TorqueCurrentFOC (Phoenix Pro). Mantiene torque constante
 *       aunque la pelota oponga resistencia al entrar al flywheel.</li>
 * </ul>
 *
 * <h2>Optimizaciones de CAN</h2>
 * <ul>
 *   <li>Signals cacheadas — se llaman una vez en el constructor, no en cada ciclo.</li>
 *   <li>{@code setUpdateFrequencyForAll} — frecuencia configurada por subsistema.</li>
 *   <li>{@code optimizeBusUtilization} — desactiva signals no usadas en cada motor.</li>
 *   <li>{@code BaseStatusSignal.refreshAll} — un solo refresh al inicio de periodic().</li>
 * </ul>
 *
 * <h2>Flujo de disparo</h2>
 * <ol>
 *   <li>El comando llama {@link #setFlywheelRPM} y {@link #setHoodAngle} al mismo tiempo.</li>
 *   <li>Cuando {@link #isReadyToShoot()} es {@code true}, el comando activa el indexer.</li>
 *   <li>Al terminar, el comando llama {@link #stopIndexer()}, {@link #coastFlywheel()},
 *       y {@link #homeHood()}.</li>
 * </ol>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Shooter/LeftRPM           — velocidad actual flywheel izquierdo [RPM]
 * Shooter/RightRPM          — velocidad actual flywheel derecho [RPM]
 * Shooter/TargetRPM         — velocidad objetivo [RPM]
 * Shooter/RPMError          — error absoluto de velocidad [RPM]
 * Shooter/LeftRotations     — posicion acumulada flywheel izquierdo [rotaciones del mecanismo]
 * Shooter/RightRotations    — posicion acumulada flywheel derecho [rotaciones del mecanismo]
 * Shooter/HoodAngleDeg      — angulo actual del hood [grados]
 * Shooter/HoodRotations     — posicion del hood [rotaciones del mecanismo]
 * Shooter/TargetAngleDeg    — angulo objetivo del hood [grados]
 * Shooter/IndexerRPS        — velocidad angular del indexer [RPS]
 * Shooter/IndexerRotations  — posicion acumulada del indexer [rotaciones del mecanismo]
 * Shooter/IndexerUnjamming  — true si el desatasco automatico del indexer esta activo
 * Shooter/LeftStatorAmps    — corriente de stator flywheel izquierdo [A]
 * Shooter/RightStatorAmps   — corriente de stator flywheel derecho [A]
 * Shooter/HoodStatorAmps    — corriente de stator hood [A]
 * Shooter/IndexerStatorAmps — corriente de stator indexer [A]
 * Shooter/FlywheelAtSpeed   — true si ambos flywheels estan en RPM objetivo
 * Shooter/HoodAtAngle       — true si el hood esta en angulo objetivo
 * Shooter/HoodAtHome        — true si el hood esta en posicion home
 * Shooter/SpinningUp        — true si flywheel esta en aceleracion (targetRPM > 0, no a velocidad)
 * Shooter/IndexerRunning    — true si el indexer esta activo
 * Shooter/ReadyToShoot      — true si flywheel + hood listos para disparar
 * </pre>
 */
public class ShooterSubsystem extends SubsystemBase {

    // =========================================================================
    // Hardware
    // =========================================================================

    // TalonFX (Phoenix 6)
    private final TalonFX leftFlywheel;
    private final TalonFX rightFlywheel;
    private final TalonFX indexerMotor;
    private final TalonFX hopperMotor;

    // SparkMax (REVLib) — hood motor
    private final SparkMax                  hoodMotor;
    private final RelativeEncoder           hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoopController;

    // =========================================================================
    // Signals cacheadas (se crean una vez, se refrescan en periodic)
    // Solo para motores TalonFX — el hood SparkMax se lee directamente
    // =========================================================================

    @SuppressWarnings("rawtypes")
    private final StatusSignal leftVelocitySignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal rightVelocitySignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal leftPositionSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal rightPositionSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal indexerPositionSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal indexerVelocitySignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal leftStatorSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal rightStatorSignal;
    @SuppressWarnings("rawtypes")
    private final StatusSignal indexerStatorSignal;

    // =========================================================================
    // Requests de control TalonFX (reusables — evita allocar en cada ciclo)
    // =========================================================================

    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0)
        .withSlot(0).withEnableFOC(true);

    // TorqueCurrentFOC para el indexer: mantiene torque [A] aunque la pelota
    // oponga resistencia. Mucho mas consistente que DutyCycleOut bajo carga variable.
    private final TorqueCurrentFOC   indexerRequest  = new TorqueCurrentFOC(0.0);
    private final NeutralOut         neutralRequest  = new NeutralOut();

    // DutyCycleOut para el hopper — transporte simple, no requiere torque preciso.
    private final DutyCycleOut       hopperDutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);

    /**
     * Request de salida de ciclo de trabajo reutilizable para control manual.
     * Se usa exclusivamente en los metodos runXxxManual() para no contaminar
     * los requests de control automatico.
     */
    private final DutyCycleOut       manualDutyCycleRequest = new DutyCycleOut(0);

    // =========================================================================
    // Estado interno
    // =========================================================================

    private double  targetRPM      = 0.0;
    private double  targetAngleDeg = ShooterConstants.HOOD_HOME_ANGLE_DEGREES;
    private boolean indexerRunning = false;

    // --- Estado del hood SparkMax ---
    private double  hoodTargetRotations = 0.0;   // posicion objetivo [rotaciones del mecanismo]
    private boolean hoodManualMode      = false;
    private double  hoodManualOutput    = 0.0;

    // --- Estado del desatasco automatico del indexer ---
    // El indexer monitorea su velocidad. Si cae a cero mientras esta activo
    // mas de INDEXER_UNJAM_TRIGGER_SECONDS, aplica una rafaga inversa y retoma.

    /** Estados posibles del indexer. */
    private enum IndexerState { STOPPED, RUNNING, UNJAMMING }
    private IndexerState indexerState = IndexerState.STOPPED;

    /** Timer para confirmar atasco del indexer (evita falsos positivos en arranque). */
    private final Timer indexerJamDetectTimer = new Timer();
    /** Timer para controlar la duracion de la reversa del indexer. */
    private final Timer indexerUnjamTimer     = new Timer();
    /** true cuando indexerJamDetectTimer esta corriendo activamente. */
    private boolean     indexerJamTimerActive = false;

    // =========================================================================
    // Constructor
    // =========================================================================

    public ShooterSubsystem() {
        // TalonFX (CAN bus Phoenix 6)
        leftFlywheel  = new TalonFX(ShooterConstants.SHOOTER_LEFT_MOTOR_ID,  TunerConstants.kCANBus);
        rightFlywheel = new TalonFX(ShooterConstants.SHOOTER_RIGHT_MOTOR_ID, TunerConstants.kCANBus);
        indexerMotor  = new TalonFX(ShooterConstants.INDEXER_MOTOR_ID,       TunerConstants.kCANBus);
        hopperMotor   = new TalonFX(HopperConstants.HOPPER_MOTOR_ID,         TunerConstants.kCANBus);

        // SparkMax (REVLib) — hood
        hoodMotor            = new SparkMax(ShooterConstants.SHOOTER_ANGLE_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder          = hoodMotor.getEncoder();
        hoodClosedLoopController = hoodMotor.getClosedLoopController();

        configureFlywheel();
        configureHood();
        configureIndexer();
        configureHopper();

        // Cachear signals TalonFX — se crean una vez y se reusan en cada ciclo.
        // El hood SparkMax se lee directamente via hoodEncoder.getPosition().
        leftVelocitySignal    = leftFlywheel.getVelocity();
        rightVelocitySignal   = rightFlywheel.getVelocity();
        leftPositionSignal    = leftFlywheel.getPosition();
        rightPositionSignal   = rightFlywheel.getPosition();
        indexerPositionSignal = indexerMotor.getPosition();
        indexerVelocitySignal = indexerMotor.getVelocity();
        leftStatorSignal      = leftFlywheel.getStatorCurrent();
        rightStatorSignal     = rightFlywheel.getStatorCurrent();
        indexerStatorSignal   = indexerMotor.getStatorCurrent();

        // Configurar frecuencias de actualizacion de las signals TalonFX usadas.
        // IMPORTANTE: llamar setUpdateFrequencyForAll ANTES de optimizeBusUtilization.
        BaseStatusSignal.setUpdateFrequencyForAll(
            ShooterConstants.FLYWHEEL_SIGNAL_HZ,
            leftVelocitySignal,  rightVelocitySignal,
            leftPositionSignal,  rightPositionSignal,
            leftStatorSignal,    rightStatorSignal
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            ShooterConstants.FLYWHEEL_SIGNAL_HZ,
            indexerPositionSignal, indexerVelocitySignal, indexerStatorSignal
        );

        // Desactivar signals no usadas en cada TalonFX — reduce trafico CAN
        leftFlywheel.optimizeBusUtilization();
        rightFlywheel.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
        hopperMotor.optimizeBusUtilization();
    }

    // =========================================================================
    // Configuracion de motores
    // =========================================================================

    private void configureFlywheel() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // PID + feedforward para control de velocidad (VelocityVoltage + FOC)
        // kP: corrector de error de velocidad
        // kV: feedforward de velocidad (1/kV_motor ~ voltios por RPS)
        // kS: voltaje minimo para superar friccion estatica
        // kA: feedforward de aceleracion (reducir lag al arrancar)
        cfg.Slot0 = new Slot0Configs()
            .withKP(ShooterConstants.FLYWHEEL_P)
            .withKI(ShooterConstants.FLYWHEEL_I)
            .withKD(ShooterConstants.FLYWHEEL_D)
            .withKV(ShooterConstants.FLYWHEEL_V)
            .withKS(ShooterConstants.FLYWHEEL_S)
            .withKA(ShooterConstants.FLYWHEEL_A);

        // Gear ratio: rotaciones del motor por rotacion de la rueda del flywheel.
        // Si hay reduccion mecanica entre el Kraken y la rueda, configurar aqui.
        cfg.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

        cfg.MotorOutput.NeutralMode = ShooterConstants.SHOOTER_NEUTRAL_MODE;

        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor izquierdo — usa la constante de inversion (SHOOTER_LEFT_INVERTED = true → CW_Positive)
        cfg.MotorOutput.Inverted = ShooterConstants.SHOOTER_LEFT_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        leftFlywheel.getConfigurator().apply(cfg);

        // Motor derecho — inversion opuesta al izquierdo para que ambos empujen en la misma direccion
        cfg.MotorOutput.Inverted = ShooterConstants.SHOOTER_RIGHT_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        rightFlywheel.getConfigurator().apply(cfg);
    }

    private void configureHood() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.inverted(ShooterConstants.SHOOTER_ANGLE_INVERTED)
           .idleMode(ShooterConstants.HOOD_IDLE_MODE)
           .smartCurrentLimit(ShooterConstants.HOOD_CURRENT_LIMIT);

        // Factor de conversion: rotaciones del motor → rotaciones del mecanismo.
        // Con velocityConversionFactor las unidades del encoder son mecanismo RPS.
        cfg.encoder
           .positionConversionFactor(1.0 / ShooterConstants.HOOD_GEAR_RATIO)
           .velocityConversionFactor(1.0 / (ShooterConstants.HOOD_GEAR_RATIO * 60.0));

        // PID onboard SparkMax (Slot 0)
        cfg.closedLoop
           .pid(ShooterConstants.HOOD_P, ShooterConstants.HOOD_I, ShooterConstants.HOOD_D)
           .outputRange(-1.0, 1.0);

        // MAXMotion: perfil de movimiento suave — equivalente a MotionMagic.
        // Disponible en SparkMax desde REVLib 2025+.
        // Velocidad en mecanismo RPS (post velocityConversionFactor),
        // aceleracion en mecanismo RPS/s, error tolerado en rotaciones del mecanismo.
        cfg.closedLoop.maxMotion
           .cruiseVelocity(ShooterConstants.HOOD_CRUISE_VELOCITY_RPS)
           .maxAcceleration(ShooterConstants.HOOD_ACCELERATION_RPS2)
           .allowedProfileError(ShooterConstants.HOOD_ANGLE_TOLERANCE_DEGREES / 360.0);

        hoodMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder a cero al arrancar — el robot debe iniciar con el hood en posicion home.
        hoodEncoder.setPosition(0.0);

        // Ir a home al iniciar
        homeHood();
    }

    private void configureIndexer() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = ShooterConstants.INDEXER_NEUTRAL_MODE;
        cfg.MotorOutput.Inverted    = ShooterConstants.INDEXER_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // StatorCurrentLimit: capa de seguridad hardware por encima del peak de TorqueCurrentFOC
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.INDEXER_STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // TorqueCurrentConfigs: limites operativos para el modo TorqueCurrentFOC.
        // El indexer usa torque control para alimentar pelotas con fuerza constante
        // incluso cuando la pelota esta entrando al flywheel (carga transitoria alta).
        cfg.TorqueCurrent = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(ShooterConstants.INDEXER_PEAK_FORWARD_AMPS)
            .withPeakReverseTorqueCurrent(ShooterConstants.INDEXER_PEAK_REVERSE_AMPS);

        indexerMotor.getConfigurator().apply(cfg);
    }

    private void configureHopper() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = HopperConstants.HOPPER_NEUTRAL_MODE;
        cfg.MotorOutput.Inverted    = HopperConstants.HOPPER_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        hopperMotor.getConfigurator().apply(cfg);
    }

    // =========================================================================
    // periodic() — cada ~20 ms
    // =========================================================================

    @Override
    public void periodic() {
        // Refrescar todas las signals en una sola llamada al bus CAN
        BaseStatusSignal.refreshAll(
            leftVelocitySignal,    rightVelocitySignal,
            leftPositionSignal,    rightPositionSignal,
            indexerPositionSignal, indexerVelocitySignal,
            leftStatorSignal,      rightStatorSignal,
            indexerStatorSignal
        );

        // Hood SparkMax PID / manual control
        if (hoodManualMode) {
            hoodMotor.set(hoodManualOutput);
        } else {
            double gravityFF = ShooterConstants.HOOD_GRAVITY_FF_VOLTS
                * Math.cos(Math.toRadians(hoodEncoder.getPosition() * 360.0));
            hoodClosedLoopController.setSetpoint(
                hoodTargetRotations,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                gravityFF,
                ArbFFUnits.kVoltage
            );
        }

        double leftRPS      = leftVelocitySignal.getValueAsDouble();
        double rightRPS     = rightVelocitySignal.getValueAsDouble();
        double leftRPM      = leftRPS  * 60.0;
        double rightRPM     = rightRPS * 60.0;
        double hoodDeg      = hoodEncoder.getPosition() * 360.0;
        double rpmError     = (targetRPM > 0)
            ? Math.abs(((leftRPM + rightRPM) / 2.0) - targetRPM)
            : 0.0;

        boolean atSpeed  = isAtTargetRPMCached(leftRPS, rightRPS);
        boolean atAngle  = isAtTargetAngleCached(hoodDeg);

        SmartDashboard.putNumber ("Shooter/LeftRPM",            leftRPM);
        SmartDashboard.putNumber ("Shooter/RightRPM",           rightRPM);
        SmartDashboard.putNumber ("Shooter/TargetRPM",          targetRPM);
        SmartDashboard.putNumber ("Shooter/RPMError",           rpmError);
        SmartDashboard.putNumber ("Shooter/LeftRotations",      leftPositionSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/RightRotations",     rightPositionSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/HoodAngleDeg",       hoodDeg);
        SmartDashboard.putNumber ("Shooter/HoodRotations",      hoodEncoder.getPosition());
        SmartDashboard.putNumber ("Shooter/TargetAngleDeg",     targetAngleDeg);
        // Evaluar desatasco automatico del indexer antes de publicar estado
        updateIndexerUnjamLogic();

        SmartDashboard.putNumber ("Shooter/IndexerRPS",         indexerVelocitySignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/IndexerRotations",   indexerPositionSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/LeftStatorAmps",     leftStatorSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/RightStatorAmps",    rightStatorSignal.getValueAsDouble());
        SmartDashboard.putNumber ("Shooter/HoodStatorAmps",     hoodMotor.getOutputCurrent());
        SmartDashboard.putNumber ("Shooter/IndexerStatorAmps",  indexerStatorSignal.getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/FlywheelAtSpeed",    atSpeed);
        SmartDashboard.putBoolean("Shooter/HoodAtAngle",        atAngle);
        SmartDashboard.putBoolean("Shooter/HoodAtHome",
            Math.abs(hoodDeg - ShooterConstants.HOOD_HOME_ANGLE_DEGREES)
                < ShooterConstants.HOOD_ANGLE_TOLERANCE_DEGREES);
        SmartDashboard.putBoolean("Shooter/SpinningUp",         targetRPM > 0 && !atSpeed);
        SmartDashboard.putBoolean("Shooter/IndexerRunning",    indexerRunning);
        SmartDashboard.putBoolean("Shooter/IndexerUnjamming",  indexerState == IndexerState.UNJAMMING);
        SmartDashboard.putBoolean("Shooter/ReadyToShoot",      atSpeed && atAngle);
    }

    // =========================================================================
    // Control del flywheel
    // =========================================================================

    /**
     * Establece el objetivo de velocidad del flywheel y lo pone en control activo.
     *
     * @param rpm RPM objetivo (tipicamente de {@link ShooterMath.ShooterResult#flywheelRPM()}).
     */
    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0;
        leftFlywheel.setControl(flywheelRequest.withVelocity(rps));
        rightFlywheel.setControl(flywheelRequest.withVelocity(rps));
    }

    /**
     * Deja los flywheels en coast (modo neutro).
     * Llamar al terminar un disparo — los flywheels desaceleran libremente.
     */
    public void coastFlywheel() {
        targetRPM = 0.0;
        leftFlywheel.setControl(neutralRequest);
        rightFlywheel.setControl(neutralRequest);
    }

    /**
     * {@code true} si ambos flywheels estan dentro de la tolerancia de RPM objetivo.
     * Usa las signals ya refrescadas en periodic().
     */
    public boolean isAtTargetRPM() {
        if (targetRPM <= 0) return false;
        return isAtTargetRPMCached(
            leftVelocitySignal.getValueAsDouble(),
            rightVelocitySignal.getValueAsDouble()
        );
    }

    private boolean isAtTargetRPMCached(double leftRPS, double rightRPS) {
        if (targetRPM <= 0) return false;
        double targetRPS    = targetRPM / 60.0;
        double toleranceRPS = ShooterConstants.FLYWHEEL_RPM_TOLERANCE / 60.0;
        return Math.abs(leftRPS  - targetRPS) < toleranceRPS
            && Math.abs(rightRPS - targetRPS) < toleranceRPS;
    }

    // =========================================================================
    // Control del hood (angulo de disparo)
    // =========================================================================

    /**
     * Mueve el hood al angulo dado usando MotionMagic (movimiento suave con compensacion de gravedad).
     *
     * @param degrees Angulo objetivo en grados desde horizontal.
     *                Debe estar en [MIN_SHOT_ANGLE, MAX_SHOT_ANGLE].
     */
    public void setHoodAngle(double degrees) {
        targetAngleDeg      = degrees;
        hoodTargetRotations = degrees / 360.0;
        hoodManualMode      = false;
    }

    /** Mueve el hood a la posicion home (descanso / tiro por defecto). */
    public void homeHood() {
        setHoodAngle(ShooterConstants.HOOD_HOME_ANGLE_DEGREES);
    }

    /**
     * {@code true} si el hood esta dentro de la tolerancia del angulo objetivo.
     * Usa la signal ya refrescada en periodic().
     */
    public boolean isAtTargetAngle() {
        return isAtTargetAngleCached(hoodEncoder.getPosition() * 360.0);
    }

    private boolean isAtTargetAngleCached(double currentDeg) {
        return Math.abs(currentDeg - targetAngleDeg) < ShooterConstants.HOOD_ANGLE_TOLERANCE_DEGREES;
    }

    // =========================================================================
    // Control del indexer
    // =========================================================================

    /**
     * Activa el indexer para alimentar la pelota al flywheel.
     *
     * <p>Usa TorqueCurrentFOC: el motor mantiene {@code INDEXER_RUN_AMPS} de torque aunque
     * la pelota oponga resistencia al entrar al flywheel (carga transitoria alta).
     * El controlador aumenta el voltaje automaticamente para mantener la corriente objetivo.</p>
     */
    public void runIndexer() {
        indexerRunning = true;
        // Solo transicionar a RUNNING si no estabamos ya en ciclo de desatasco
        if (indexerState == IndexerState.STOPPED) {
            indexerState = IndexerState.RUNNING;
        }
        indexerMotor.setControl(indexerRequest.withOutput(ShooterConstants.INDEXER_RUN_AMPS));
        hopperMotor.setControl(hopperDutyCycleRequest.withOutput(HopperConstants.HOPPER_DUTY_CYCLE));
    }

    /** Detiene el indexer y el hopper (NeutralOut — freno segun INDEXER_NEUTRAL_MODE). */
    public void stopIndexer() {
        indexerRunning        = false;
        indexerState          = IndexerState.STOPPED;
        indexerJamTimerActive = false;
        indexerMotor.setControl(neutralRequest);
        hopperMotor.setControl(neutralRequest);
    }

    /** {@code true} si el indexer esta actualmente activo. */
    public boolean isIndexerRunning() {
        return indexerRunning;
    }

    // =========================================================================
    // Estado compuesto
    // =========================================================================

    /**
     * {@code true} si el flywheel esta a velocidad Y el hood esta en angulo.
     * Esta es la condicion para activar el indexer en {@link ShootCommand}.
     */
    public boolean isReadyToShoot() {
        return isAtTargetRPM() && isAtTargetAngle();
    }

    // =========================================================================
    // Control manual (por motor individual) — usar solo desde ManualXxxCommand
    // =========================================================================

    /**
     * Aplica un porcentaje de salida directo a AMBOS flywheels simultaneamente.
     * La inversion de cada motor (configurada en configureFlywheel) hace que
     * giren en direcciones opuestas fisicamente para empujar la pelota hacia afuera.
     * La salida se clampea a [-MAX, +MAX].
     *
     * @param percent Salida en [-1.0, 1.0]. Positivo = lanzar, negativo = reversa.
     */
    public void runFlywheelManual(double percent) {
        double clamped = clampManual(percent, ShooterConstants.FLYWHEEL_MANUAL_MAX_OUTPUT);
        leftFlywheel.setControl(manualDutyCycleRequest.withOutput(clamped));
        rightFlywheel.setControl(manualDutyCycleRequest.withOutput(clamped));
    }

    /**
     * Detiene ambos flywheels y los deja en coast.
     */
    public void stopFlywheelManual() {
        leftFlywheel.setControl(neutralRequest);
        rightFlywheel.setControl(neutralRequest);
    }

    /**
     * Aplica un porcentaje de salida directo al motor del hood (sin MotionMagic/PID).
     * Usar con precaucion — no hay limites de posicion activos.
     * La salida se clampea a [-MAX, +MAX].
     *
     * @param percent Salida en [-1.0, 1.0]. Se clampea a HOOD_MANUAL_MAX_OUTPUT.
     */
    public void runHoodManual(double percent) {
        hoodManualOutput = clampManual(percent, ShooterConstants.HOOD_MANUAL_MAX_OUTPUT);
        hoodManualMode   = true;
    }

    /**
     * Detiene el hood y congela la posicion actual con PID.
     */
    public void stopHoodManual() {
        hoodTargetRotations = hoodEncoder.getPosition();
        targetAngleDeg      = hoodTargetRotations * 360.0;
        hoodManualOutput    = 0.0;
        hoodManualMode      = false;
    }

    /**
     * Aplica una corriente de torque directa al indexer (sin automatismo).
     * Usar solo desde {@code ManualIndexerCommand}.
     * La corriente se clampea a [-MAX, +MAX] para pruebas seguras.
     *
     * @param amps Corriente objetivo [A]. Se clampea a INDEXER_MANUAL_MAX_AMPS.
     *             Positivo = alimentar hacia flywheel, negativo = reversa.
     */
    public void runIndexerManual(double amps) {
        // En modo manual el desatasco automatico se desactiva — el operador tiene control total.
        indexerState          = IndexerState.STOPPED;
        indexerJamTimerActive = false;
        double clamped = clampManual(amps, ShooterConstants.INDEXER_MANUAL_MAX_AMPS);
        indexerMotor.setControl(indexerRequest.withOutput(clamped));
        // Hopper sigue al indexer: misma direccion que el trigger, salida fija de diagnostico.
        double hopperOut = clampManual(
            Math.signum(amps) * HopperConstants.HOPPER_MANUAL_MAX_OUTPUT,
            HopperConstants.HOPPER_MANUAL_MAX_OUTPUT
        );
        hopperMotor.setControl(hopperDutyCycleRequest.withOutput(hopperOut));
    }

    /**
     * Detiene el indexer y el hopper (NeutralOut).
     */
    public void stopIndexerManual() {
        indexerState          = IndexerState.STOPPED;
        indexerJamTimerActive = false;
        indexerMotor.setControl(neutralRequest);
        hopperMotor.setControl(neutralRequest);
    }

    /**
     * Clampea la salida manual a [-maxAbs, +maxAbs].
     *
     * @param percent Valor de entrada
     * @param maxAbs  Limite absoluto (positivo)
     * @return Valor clampeado
     */
    private static double clampManual(double percent, double maxAbs) {
        return Math.max(-maxAbs, Math.min(maxAbs, percent));
    }

    // =========================================================================
    // Logica de desatasco automatico del indexer
    // =========================================================================

    /**
     * Maquina de estados para el desatasco automatico del indexer.
     *
     * <p>Llamado desde {@link #periodic()} cada 20 ms. Solo actua cuando
     * {@code indexerState} es {@code RUNNING} o {@code UNJAMMING}.</p>
     *
     * <ul>
     *   <li><b>RUNNING</b>: monitorea velocidad. Si cae bajo el umbral mas de
     *       {@code INDEXER_UNJAM_TRIGGER_SECONDS}, transiciona a {@code UNJAMMING}.</li>
     *   <li><b>UNJAMMING</b>: aplica corriente inversa. Tras
     *       {@code INDEXER_UNJAM_REVERSE_SECONDS}, vuelve a {@code RUNNING}.</li>
     * </ul>
     */
    private void updateIndexerUnjamLogic() {
        if (indexerState == IndexerState.STOPPED) {
            return;
        }

        double velocityRPS = indexerVelocitySignal.getValueAsDouble();

        if (indexerState == IndexerState.RUNNING) {
            boolean isStalled = Math.abs(velocityRPS) < ShooterConstants.INDEXER_JAM_VELOCITY_THRESHOLD_RPS;

            if (isStalled) {
                if (!indexerJamTimerActive) {
                    // Primera vez que detectamos velocidad baja — arrancar el timer
                    indexerJamDetectTimer.restart();
                    indexerJamTimerActive = true;
                } else if (indexerJamDetectTimer.hasElapsed(ShooterConstants.INDEXER_UNJAM_TRIGGER_SECONDS)) {
                    // Atasco confirmado — iniciar reversa
                    indexerState          = IndexerState.UNJAMMING;
                    indexerJamTimerActive = false;
                    indexerUnjamTimer.restart();
                    indexerMotor.setControl(indexerRequest.withOutput(ShooterConstants.INDEXER_UNJAM_REVERSE_AMPS));
                }
            } else {
                // Indexer girando normalmente — resetear timer de deteccion
                indexerJamTimerActive = false;
            }

        } else if (indexerState == IndexerState.UNJAMMING) {
            if (indexerUnjamTimer.hasElapsed(ShooterConstants.INDEXER_UNJAM_REVERSE_SECONDS)) {
                // Reversa completada — retomar alimentacion
                indexerState = IndexerState.RUNNING;
                indexerMotor.setControl(indexerRequest.withOutput(ShooterConstants.INDEXER_RUN_AMPS));
            }
        }
    }
}

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Drive.TunerConstants;

/**
 * Archivo central de constantes del robot Horus — Equipo 6348.
 *
 * Cada subsistema tiene su propia clase interna con sus constantes.
 * Convenciones:
 *   - Constantes: UPPER_SNAKE_CASE
 *   - Sin prefijos k ni m
 *   - Nombres completamente explicitos
 *
 * Configuracion de campo desde SmartDashboard:
 *   Preferences → IsAndymarkField = true/false → reiniciar robot
 */
public final class Constants {

    // =========================================================================
    // Field (Geometria del campo FRC 2026)
    // =========================================================================

    /**
     * Geometria del campo FRC 2026 — dos variantes: Welded y Andymark.
     *
     * <h2>Como cambiar el tipo de campo</h2>
     * <ol>
     *   <li>Abrir SmartDashboard o Shuffleboard.</li>
     *   <li>Ir a la tabla <b>Preferences</b>.</li>
     *   <li>Cambiar <b>IsAndymarkField</b> a {@code true} (Andymark) o {@code false} (Welded).</li>
     *   <li><b>Reiniciar el codigo del robot</b> para que el cambio tome efecto.</li>
     * </ol>
     *
     * Origen del campo: esquina Red (x=0, y=0) segun convencion WPILib.
     * Eje X: de alianza Red hacia alianza Blue (longitud del campo).
     * Eje Y: de la pared sur hacia la pared norte.
     *
     * <h2>Zonas del campo</h2>
     * <ul>
     *   <li><b>RED_ALLIANCE</b>:  x &lt; RED_ZONE_MAX_X</li>
     *   <li><b>NEUTRAL</b>:       RED_ZONE_MAX_X &le; x &le; BLUE_ZONE_MIN_X</li>
     *   <li><b>BLUE_ALLIANCE</b>: x &gt; BLUE_ZONE_MIN_X</li>
     * </ul>
     */
    public static final class FieldConstants {

        public enum FieldType { WELDED, ANDYMARK }

        /**
         * Tipo de campo activo — determinado desde Preferences al arrancar.
         * Preferences key: {@code IsAndymarkField} (boolean).
         * Default: {@code false} → Welded.
         *
         * <p>Requiere reinicio del robot para aplicar cambios.</p>
         */
        public static final FieldType ACTIVE_FIELD_TYPE;

        static {
            // initBoolean crea la clave con el valor por defecto si no existe.
            // Si ya existe (guardada en flash del RoboRIO), no la sobrescribe.
            Preferences.initBoolean("IsAndymarkField", false);
            ACTIVE_FIELD_TYPE = Preferences.getBoolean("IsAndymarkField", false)
                ? FieldType.ANDYMARK : FieldType.WELDED;
        }

        // =====================================================================
        // Dimensiones brutas — convertidas de pulgadas a metros (1" = 0.0254 m)
        // =====================================================================

        // --- Welded field: 651.22" x 317.69" ---
        public static final double W_LEN =  651.22 * 0.0254; // 16.5410 m
        public static final double W_WID =  317.69 * 0.0254; //  8.0693 m
        // Scoring target X desde cada borde [182.11"]
        public static final double W_SCR =  182.11 * 0.0254; //  4.6256 m
        // Passing target X desde cada borde [91.00"]
        public static final double W_PSX =   91.00 * 0.0254; //  2.3114 m
        // Passing target Y desde cada borde [79.42"]
        public static final double W_PSY =   79.42 * 0.0254; //  2.0173 m

        // --- Andymark field: 650.12" x 316.64" ---
        public static final double A_LEN =  650.12 * 0.0254; // 16.5130 m
        public static final double A_WID =  316.64 * 0.0254; //  8.0427 m
        // Scoring target X desde cada borde [181.56"]
        public static final double A_SCR =  181.56 * 0.0254; //  4.6116 m
        // Passing target X desde cada borde [91.00"] — igual que Welded
        public static final double A_PSX =   91.00 * 0.0254; //  2.3114 m
        // Passing target Y desde cada borde [79.42"] — igual que Welded
        public static final double A_PSY =   79.42 * 0.0254; //  2.0173 m

        // =====================================================================
        // Dimensiones activas (seleccionadas por ACTIVE_FIELD_TYPE)
        // =====================================================================
        // IMPORTANTE: estas constantes deben declararse DESPUES del bloque static{}
        // para que ACTIVE_FIELD_TYPE ya este inicializado cuando sel() se llama.

        /** Longitud total del campo (eje X) [m]. */
        public static final double FIELD_LENGTH = sel(W_LEN, A_LEN);

        /** Ancho total del campo (eje Y) [m]. */
        public static final double FIELD_WIDTH  = sel(W_WID, A_WID);

        // --- Limites de zona ---

        /** X maximo de la zona Red Alliance [m]. Todo x < este valor es zona roja. */
        public static final double RED_ZONE_MAX_X  = sel(W_SCR, A_SCR);

        /** X minimo de la zona Blue Alliance [m]. Todo x > este valor es zona azul. */
        public static final double BLUE_ZONE_MIN_X = FIELD_LENGTH - RED_ZONE_MAX_X;

        // --- Scoring targets (simetricos, centrados en Y) ---

        public static final double RED_SCORING_X  = sel(W_SCR, A_SCR);
        public static final double RED_SCORING_Y  = FIELD_WIDTH / 2.0;
        public static final double BLUE_SCORING_X = FIELD_LENGTH - RED_SCORING_X;
        public static final double BLUE_SCORING_Y = FIELD_WIDTH  / 2.0;

        // --- Passing targets ---
        // PASS_1 = lado sur (Y bajo); PASS_2 = lado norte (Y alto)

        public static final double RED_PASS_X    = sel(W_PSX, A_PSX);
        public static final double BLUE_PASS_X   = FIELD_LENGTH - RED_PASS_X;
        public static final double PASS_Y_SOUTH  = sel(W_PSY, A_PSY);
        public static final double PASS_Y_NORTH  = FIELD_WIDTH - PASS_Y_SOUTH;

        // =====================================================================
        // Selector interno
        // =====================================================================

        public static double sel(double welded, double andymark) {
            return (ACTIVE_FIELD_TYPE == FieldType.WELDED) ? welded : andymark;
        }
    }

    // =========================================================================
    // Drive (Swerve Drivetrain)
    // =========================================================================

    /**
     * Constantes de comportamiento del drivetrain swerve.
     *
     * Las constantes de hardware (IDs, gear ratios, offsets de encoder) permanecen
     * en TunerConstants.java porque ese archivo es generado por CTRE Tuner X.
     * Las constantes de tuning y geometria se definen aqui para ser editables desde
     * un solo lugar sin tener que tocar el archivo generado.
     */
    public static final class DriveConstants {

        /** Velocidad angular maxima en teleop (0.75 rot/s) [rad/s]. */
        public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /** Deadband del joystick: porcentaje de la velocidad maxima ignorado (5%). */
        public static final double JOYSTICK_DEADBAND_PERCENT = 0.05;

        /**
         * Frecuencia de actualizacion de odometria [Hz].
         *
         * El drivetrain Phoenix 6 soporta hasta 250 Hz en CANivore.
         * Mayor frecuencia = mejor precision de posicion, especialmente a alta velocidad.
         * CANivore: hasta 250 Hz. CAN estandar: hasta 100 Hz.
         */
        public static final double ODOMETRY_UPDATE_HZ = 250.0;

        /**
         * Periodo del hilo de simulacion del swerve [s].
         * 0.004 s = 250 Hz — coincide con la frecuencia de odometria en hardware real.
         * Solo activo en modo simulacion (sin hardware real conectado).
         */
        public static final double SIM_LOOP_PERIOD_SECONDS = 0.004;

        // --- Voltajes para rutinas SysId ---
        // Usados en CommandSwerveDrivetrain para caracterizar los motores.
        // SysId aplica rampas y pasos de voltaje para medir ganancias mecanicas.

        /** Voltaje de paso para la rutina SysId de traslacion (drive) [V]. */
        public static final double SYSID_TRANSLATION_STEP_VOLTS = 4.0;

        /** Voltaje de paso para la rutina SysId de giro de modulo (steer) [V]. */
        public static final double SYSID_STEER_STEP_VOLTS = 7.0;

        /** Tasa de rampa para la rutina SysId de rotacion del chasis [V/s]. */
        public static final double SYSID_ROTATION_RAMP_RATE_VOLTS_PER_S = Math.PI / 6.0;

        /** Voltaje de paso para la rutina SysId de rotacion del chasis [V]. */
        public static final double SYSID_ROTATION_STEP_VOLTS = Math.PI;

        // --- PID PathPlanner — Traslacion ---
        // TODO: Tunear en robot fisico con PathPlanner GUI o SysId
        public static final double PATHPLANNER_TRANSLATION_P = 5.0;
        public static final double PATHPLANNER_TRANSLATION_I = 0.0;
        public static final double PATHPLANNER_TRANSLATION_D = 0.0;

        // --- PID PathPlanner — Rotacion ---
        // TODO: Tunear en robot fisico
        public static final double PATHPLANNER_ROTATION_P = 5.0;
        public static final double PATHPLANNER_ROTATION_I = 0.0;
        public static final double PATHPLANNER_ROTATION_D = 0.0;

        // --- PID Auto-aim (FieldCentricFacingAngle — heading controller de CTRE) ---
        // Controla el omega del robot para apuntar al objetivo calculado por ShooterMath.
        // TODO: Tunear en robot fisico. Subir P si el giro es lento, bajar si oscila.
        public static final double AUTO_AIM_P = 5.0;
        public static final double AUTO_AIM_I = 0.0;
        public static final double AUTO_AIM_D = 0.0;

        /** Tolerancia de heading para "AutoAim/Aligned" en SmartDashboard [grados]. */
        public static final double AUTO_AIM_TOLERANCE_DEGREES = 2.0;

        // --- Ganancias del modulo de giro (steer) ---
        // Estos valores van al Slot0Configs del motor de steer de cada modulo swerve.
        public static final double STEERING_GAINS_KP = 100.0;
        public static final double STEERING_GAINS_KI = 0.0;
        public static final double STEERING_GAINS_KD = 0.5;
        public static final double STEERING_GAINS_KS = 0.1;  // Voltaje de friccion estatica
        public static final double STEERING_GAINS_KV = 2.66; // Voltaje por RPS (1/free_speed)
        public static final double STEERING_GAINS_KA = 0.0;

        // --- Ganancias del modulo de traccion (drive) ---
        public static final double DRIVE_GAINS_KP = 0.1;
        public static final double DRIVE_GAINS_KI = 0.0;
        public static final double DRIVE_GAINS_KD = 0.0;
        public static final double DRIVE_GAINS_KS = 0.0;
        public static final double DRIVE_GAINS_KV = 0.124;
        public static final double DRIVE_GAINS_KA = 0.0;

        /** Limite de corriente de stator de los motores de drive [A]. */
        public static final double DRIVE_GAINS_STATOR_CURRENT_LIMIT = 60.0;

        // --- Geometria del tren motriz ---

        /**
         * Relacion de acoplamiento entre el motor de steer y el eje de drive.
         * Por cada rotacion del azimut, el eje de drive gira este numero de veces.
         */
        public static final double COUPLE_RATIO = 3.5714285714285716;

        /** Relacion de transmision del motor de drive al eje de rueda. */
        public static final double DRIVE_GEAR_RATIO = 8.142857142857142;

        /** Relacion de transmision del motor de steer al eje de azimut. */
        public static final double STEER_GEAR_RATIO = 21.428571428571427;

        /** Radio de la rueda omnidireccional [pulgadas → metros internamente]. */
        public static final Distance WHEEL_RADIUS = Inches.of(2);

        /** Invertir motores de drive en el lado izquierdo del robot. */
        public static final boolean INVERT_LEFT_SIDE = false;

        /** Invertir motores de drive en el lado derecho del robot. */
        public static final boolean INVERT_RIGHT_SIDE = true;

        /** Ancho del chasis (distancia entre modulos laterales) [pulgadas → metros]. */
        public static final Distance ROBOT_WIDTH  = Inches.of(23.5);

        /** Largo del chasis (distancia entre modulos frontales y traseros) [pulgadas → metros]. */
        public static final Distance ROBOT_LENGTH = Inches.of(23.5);

        // --- IDs y offsets de los 4 modulos swerve ---
        // El offset del encoder es el angulo del modulo cuando la rueda apunta
        // hacia adelante del robot. Se mide con Tuner X y se guarda aqui.

        // Modulo: Front Left (frontal izquierdo)
        public static final int     FRONT_LEFT_DRIVE_MOTOR_ID       = 10;
        public static final int     FRONT_LEFT_STEER_MOTOR_ID       = 6;
        public static final int     FRONT_LEFT_ENCODER_ID           = 2;
        public static final Angle   FRONT_LEFT_ENCODER_OFFSET       = Rotations.of(0.27197265625);
        public static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean FRONT_LEFT_ENCODER_INVERTED     = false;
        public static final Distance FRONT_LEFT_X_POS = ROBOT_WIDTH.div(2);
        public static final Distance FRONT_LEFT_Y_POS = ROBOT_LENGTH.div(2);

        // Modulo: Front Right (frontal derecho)
        public static final int     FRONT_RIGHT_DRIVE_MOTOR_ID       = 9;
        public static final int     FRONT_RIGHT_STEER_MOTOR_ID       = 5;
        public static final int     FRONT_RIGHT_ENCODER_ID           = 1;
        public static final Angle   FRONT_RIGHT_ENCODER_OFFSET       = Rotations.of(-0.30810546875);
        public static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean FRONT_RIGHT_ENCODER_INVERTED     = false;
        public static final Distance FRONT_RIGHT_X_POS = ROBOT_WIDTH.div(2);
        public static final Distance FRONT_RIGHT_Y_POS = ROBOT_LENGTH.div(-2);

        // Modulo: Back Left (trasero izquierdo)
        public static final int     BACK_LEFT_DRIVE_MOTOR_ID       = 11;
        public static final int     BACK_LEFT_STEER_MOTOR_ID       = 7;
        public static final int     BACK_LEFT_ENCODER_ID           = 3;
        public static final Angle   BACK_LEFT_ENCODER_OFFSET       = Rotations.of(-0.41455078125);
        public static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
        public static final boolean BACK_LEFT_ENCODER_INVERTED     = false;
        public static final Distance BACK_LEFT_X_POS = ROBOT_WIDTH.div(-2);
        public static final Distance BACK_LEFT_Y_POS = ROBOT_LENGTH.div(2);

        // Modulo: Back Right (trasero derecho)
        public static final int     BACK_RIGHT_DRIVE_MOTOR_ID       = 12;
        public static final int     BACK_RIGHT_STEER_MOTOR_ID       = 8;
        public static final int     BACK_RIGHT_ENCODER_ID           = 4;
        public static final Angle   BACK_RIGHT_ENCODER_OFFSET       = Rotations.of(-0.14013671875);
        public static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        public static final boolean BACK_RIGHT_ENCODER_INVERTED     = false;
        public static final Distance BACK_RIGHT_X_POS = ROBOT_WIDTH.div(-2);
        public static final Distance BACK_RIGHT_Y_POS = ROBOT_LENGTH.div(-2);

        // IMPORTANTE: este campo se declara AL FINAL para evitar una dependencia circular.
        // DriveConstants referencia TunerConstants (kSpeedAt12Volts) y TunerConstants
        // referencia DriveConstants (WHEEL_RADIUS, DRIVE_GEAR_RATIO, IDs, etc.).
        // En Java, los campos static final se inicializan en orden textual.
        // Si MAX_SPEED fuera el primer campo, TunerConstants se cargaría antes de que
        // WHEEL_RADIUS y los IDs de motor estuvieran inicializados → NullPointerException.
        // Colocandolo ultimo, todos los campos de hardware ya estan listos cuando
        // TunerConstants los consulta durante su propia inicializacion estatica.

        /**
         * Velocidad maxima de teleop derivada de TunerConstants [m/s].
         * Igual a {@code TunerConstants.kSpeedAt12Volts} convertido a m/s.
         */
        public static final double MAX_SPEED_METERS_PER_SECOND =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    // =========================================================================
    // CAN Bus
    // =========================================================================

    /** Identificadores del bus CAN del robot. */
    public static final class CAN {
        /** Nombre del CANivore — debe coincidir con el nombre configurado en Tuner X. */
        public static final String CANIVORE_NAME = "6348 Horus CANivore";
    }

    // =========================================================================
    // Shooter (Mecanismo de disparo)
    // =========================================================================

    /**
     * Constantes del subsistema de disparo.
     *
     * <h2>Hardware</h2>
     * <ul>
     *   <li>Flywheel izquierdo (ID 16) + derecho (ID 17): Kraken X60, VelocityVoltage + FOC</li>
     *   <li>Hood / angulo (ID 20): Kraken X60, MotionMagicVoltage + FOC</li>
     *   <li>Indexer (ID 22): Kraken X60, DutyCycleOut</li>
     * </ul>
     *
     * <h2>Por que VelocityVoltage para el flywheel</h2>
     * <p>VelocityVoltage + FOC es la eleccion correcta para flywheels en FRC:
     * <ul>
     *   <li>FOC mejora la eficiencia y suavidad del torque (mejor que non-FOC).</li>
     *   <li>kV + kS proporciona un feedforward preciso para alcanzar velocidad rapidamente.</li>
     *   <li>kP corrige desviaciones de carga (cuando la pelota impacta el flywheel).</li>
     *   <li>VelocityTorqueCurrentFOC ofrece mejor torque pero requiere SysId de torque
     *       (kA en Amperios), que es mas complejo de tunear. No necesario para FRC.</li>
     * </ul>
     *
     * <h2>Por que MotionMagicVoltage para el hood</h2>
     * <p>MotionMagicVoltage + FOC es ideal para mecanismos de posicion con movimiento suave:
     * <ul>
     *   <li>Genera un perfil de velocidad trapezoidal automaticamente (sin sacudidas).</li>
     *   <li>kG + GravityType.Arm_Cosine compensa la gravedad segun el angulo del hood.</li>
     *   <li>kV + kA rastrean el perfil de movimiento con precision.</li>
     * </ul>
     */
    public static final class ShooterConstants {

        // --- IDs de motores ---
        public static final int SHOOTER_LEFT_MOTOR_ID  = 16;
        public static final int SHOOTER_RIGHT_MOTOR_ID = 17;
        public static final int SHOOTER_ANGLE_MOTOR_ID = 20;
        public static final int INDEXER_MOTOR_ID       = 22;

        // --- Direccion de giro ---
        // Verificado con template mty_final: Left=CW_Positive, Right=CCW_Positive → convergen hacia la pelota.
        public static final boolean SHOOTER_LEFT_INVERTED  = true;
        public static final boolean SHOOTER_RIGHT_INVERTED = false;
        public static final boolean SHOOTER_ANGLE_INVERTED = false;
        public static final boolean INDEXER_INVERTED       = true;   // CW_Positive → alimenta hacia los flywheels

        // --- Modos neutros ---
        public static final NeutralModeValue SHOOTER_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue INDEXER_NEUTRAL_MODE  = NeutralModeValue.Coast;
        // Hood: SparkMax — usa IdleMode, no NeutralModeValue
        public static final IdleMode HOOD_IDLE_MODE = IdleMode.kBrake;

        // --- Limites de corriente (Amps) ---
        public static final double SHOOTER_STATOR_CURRENT_LIMIT = 80.0;
        public static final int    HOOD_CURRENT_LIMIT = 40;   // SparkMax smartCurrentLimit [A]
        public static final double INDEXER_STATOR_CURRENT_LIMIT = 60.0;

        // =====================================================================
        // [1/4] Constantes fisicas — TUNEAR en robot fisico
        // =====================================================================

        /** Aceleracion gravitacional estandar [m/s^2]. */
        public static final double GRAVITY = 9.80665;

        public static final double COMPENSATION_MULTIPLICATOR = 1.1;

        /**
         * Altura del punto de salida de la pelota sobre el piso [m].
         * Medir con cinta desde el piso hasta el centro del orificio del flywheel.
         */
        public static final double SHOOTER_HEIGHT_METERS = 0.60; // TODO: medir

        /**
         * Altura maxima del proyectil (apex de la parabola) [m].
         * Fijarlo hace que T_TOTAL sea constante independientemente de la distancia.
         * Debe ser mayor que TARGET_HEIGHT_METERS y SHOOTER_HEIGHT_METERS.
         */
        public static final double APEX_HEIGHT_METERS = 2.40; // TODO: ajustar

        /**
         * Altura del objetivo (embudo/speaker) sobre el piso [m].
         * Determina T_FALL y por ende T_TOTAL.
         */
        public static final double TARGET_HEIGHT_METERS = 2.10; // TODO: confirmar

        /**
         * Distancia horizontal del centro del robot a la boca del flywheel [m].
         * El shooter apunta hacia adelante del robot.
         */
        public static final double SHOOTER_X_OFFSET_METERS = 0.30; // TODO: medir

        /** Radio del flywheel en el punto de contacto con la pelota [m]. */
        public static final double FLYWHEEL_RADIUS_METERS = 0.05; // TODO: medir con calibrador

        // =====================================================================
        // Angulos del hood — relacion complementaria
        //
        // PRINCIPIO FUNDAMENTAL:
        // El angulo de salida de la pelota (theta desde la horizontal) es el
        // angulo COMPLEMENTARIO del angulo mecanico del hood.
        //
        //   angulo_salida = COMPLEMENTARY_ANGLE_DEGREES - angulo_hood
        //   angulo_hood   = COMPLEMENTARY_ANGLE_DEGREES - angulo_salida
        //
        // Ejemplo: hood a 65° mecanicos → pelota sale a 25° de la horizontal.
        // Ejemplo: se requiere 30° de salida → hood debe ir a 60° mecanicos.
        //
        // El hood NUNCA descansa en 0°. Su reposo es HOOD_MIN_ANGLE_DEGREES (minimo
        // mecanico). Su limite superior es HOOD_MAX_ANGLE_DEGREES (tope fisico).
        // Los angulos de salida MIN/MAX se DERIVAN de los angulos del hood.
        // =====================================================================

        /** Offset para calculo del angulo complementario [grados]. Siempre 90°. */
        public static final double COMPLEMENTARY_ANGLE_DEGREES = 90.0;

        /**
         * Angulo mecanico MINIMO del hood desde su referencia [grados].
         *
         * <p>Es la posicion de reposo (home) — donde el hood queda al iniciar y al finalizar
         * cada disparo. No debe ser 0° porque eso representaria disparar verticalmente.</p>
         *
         * <p>Corresponde al angulo de SALIDA maximo (tiro mas empinado, menor alcance).<br>
         * Relacion: angulo_salida_max = 90° - HOOD_MIN_ANGLE_DEGREES.</p>
         *
         * TODO: Medir con inclinometro fisico en el robot. Ajustar ANTES de operar.
         */
        public static final double HOOD_MIN_ANGLE_DEGREES = 20.0;

        /**
         * Angulo mecanico MAXIMO del hood desde su referencia [grados].
         *
         * <p>Limite fisico del mecanismo — el hood no puede ir mas alla de este angulo.
         * Sobrepasar este limite puede daniar el mecanismo o las cadenas.</p>
         *
         * <p>Corresponde al angulo de SALIDA minimo (tiro mas plano, mayor alcance).<br>
         * Relacion: angulo_salida_min = 90° - HOOD_MAX_ANGLE_DEGREES.</p>
         *
         * TODO: Medir con inclinometro fisico en el robot. Ajustar ANTES de operar.
         */
        public static final double HOOD_MAX_ANGLE_DEGREES = 70.0;

        /**
         * Angulo de salida MINIMO de la pelota desde la horizontal [grados].
         *
         * <p>Ocurre cuando el hood esta en su angulo MAXIMO (disparo mas plano, mayor alcance).
         * Derivado: 90° - HOOD_MAX_ANGLE_DEGREES. No modificar directamente.</p>
         */
        public static final double MIN_SHOT_ANGLE_DEGREES =
            COMPLEMENTARY_ANGLE_DEGREES - HOOD_MAX_ANGLE_DEGREES;

        /**
         * Angulo de salida MAXIMO de la pelota desde la horizontal [grados].
         *
         * <p>Ocurre cuando el hood esta en su angulo MINIMO/HOME (disparo mas empinado, menor alcance).
         * Derivado: 90° - HOOD_MIN_ANGLE_DEGREES. No modificar directamente.</p>
         */
        public static final double MAX_SHOT_ANGLE_DEGREES =
            COMPLEMENTARY_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES;

        /** RPM maximas del flywheel — spec. Kraken X60 [RPM]. */
        public static final double MAX_FLYWHEEL_RPM = 6000.0;

        /** Distancia maxima de disparo util [m]. */
        public static final double MAX_SHOT_DISTANCE_METERS = 8.0; // TODO: probar en pista

        // =====================================================================
        // [2/4] Constantes pre-calculadas (calculadas UNA VEZ al inicio)
        // =====================================================================

        /** Velocidad vertical inicial del proyectil [m/s]. vy0 = sqrt(2g*(h_apex - h_shooter)) */
        public static final double VY0_METERS_PER_SECOND =
            Math.sqrt(2.0 * GRAVITY * (APEX_HEIGHT_METERS - SHOOTER_HEIGHT_METERS));

        /** Tiempo de subida (shooter → apex) [s]. t_rise = vy0/g */
        public static final double T_RISE_SECONDS = VY0_METERS_PER_SECOND / GRAVITY;

        /** Tiempo de bajada (apex → objetivo) [s]. t_fall = sqrt(2*(h_apex-h_target)/g) */
        public static final double T_FALL_SECONDS =
            Math.sqrt(2.0 * (APEX_HEIGHT_METERS - TARGET_HEIGHT_METERS) / GRAVITY);

        /**
         * Tiempo total de vuelo [s] — CONSTANTE para cualquier distancia.
         * Solo depende de las alturas (shooter, apex, objetivo), no de la distancia.
         */
        public static final double T_TOTAL_SECONDS = T_RISE_SECONDS + T_FALL_SECONDS;

        /** Factor conversion velocidad → RPM. RPM = V * RPM_PER_MS. */
        public static final double RPM_PER_MS = 60.0 / (2.0 * Math.PI * FLYWHEEL_RADIUS_METERS);

        /** Velocidad maxima de la pelota limitada por el flywheel [m/s]. */
        public static final double MAX_LAUNCH_VELOCITY_MS = MAX_FLYWHEEL_RPM / RPM_PER_MS;

        /** Velocidad horizontal maxima posible dado el limite de RPM [m/s]. */
        public static final double MAX_VX_METERS_PER_SECOND =
            (VY0_METERS_PER_SECOND < MAX_LAUNCH_VELOCITY_MS)
                ? Math.sqrt(MAX_LAUNCH_VELOCITY_MS * MAX_LAUNCH_VELOCITY_MS
                            - VY0_METERS_PER_SECOND * VY0_METERS_PER_SECOND)
                : 0.0;

        /** Distancia efectiva maxima limitada por RPM [m]. */
        public static final double MAX_EFFECTIVE_DIST_BY_RPM_METERS =
            MAX_VX_METERS_PER_SECOND * T_TOTAL_SECONDS;

        /** Distancia efectiva minima limitada por angulo maximo [m]. */
        public static final double MIN_EFFECTIVE_DIST_BY_ANGLE_METERS =
            (VY0_METERS_PER_SECOND / Math.tan(Math.toRadians(MAX_SHOT_ANGLE_DEGREES)))
            * T_TOTAL_SECONDS;

        /** Distancia efectiva maxima limitada por angulo minimo [m]. */
        public static final double MAX_EFFECTIVE_DIST_BY_ANGLE_METERS =
            (VY0_METERS_PER_SECOND / Math.tan(Math.toRadians(MIN_SHOT_ANGLE_DEGREES)))
            * T_TOTAL_SECONDS;

        // =====================================================================
        // [3/4] Zonas de disparo — referencias al campo activo
        // =====================================================================

        // --- Blue Alliance ---
        public static final Translation3d BLUE_SCORING_TARGET =
            new Translation3d(FieldConstants.BLUE_SCORING_X, FieldConstants.BLUE_SCORING_Y,
                              TARGET_HEIGHT_METERS);
        public static final Translation3d BLUE_PASS_TARGET_1 =
            new Translation3d(FieldConstants.BLUE_PASS_X, FieldConstants.PASS_Y_SOUTH,
                              TARGET_HEIGHT_METERS);
        public static final Translation3d BLUE_PASS_TARGET_2 =
            new Translation3d(FieldConstants.BLUE_PASS_X, FieldConstants.PASS_Y_NORTH,
                              TARGET_HEIGHT_METERS);

        // --- Red Alliance ---
        public static final Translation3d RED_SCORING_TARGET =
            new Translation3d(FieldConstants.RED_SCORING_X, FieldConstants.RED_SCORING_Y,
                              TARGET_HEIGHT_METERS);
        public static final Translation3d RED_PASS_TARGET_1 =
            new Translation3d(FieldConstants.RED_PASS_X, FieldConstants.PASS_Y_SOUTH,
                              TARGET_HEIGHT_METERS);
        public static final Translation3d RED_PASS_TARGET_2 =
            new Translation3d(FieldConstants.RED_PASS_X, FieldConstants.PASS_Y_NORTH,
                              TARGET_HEIGHT_METERS);

        // =====================================================================
        // [4/4] Control del subsistema fisico
        // =====================================================================

        // --- Flywheel — VelocityVoltage + FOC ---
        // kP: Voltios por RPS de error. Sube si el flywheel no mantiene velocidad bajo carga.
        // kV: Voltios por RPS. Feedforward principal. Calcular: 12V / free_speed_RPS
        //     Para Kraken X60: 12V / 100 RPS = 0.12 V/RPS
        // kS: Voltios para superar friccion estatica. Subir hasta que el motor justo arranque.
        //     Tipicamente 0.1-0.5 V. Mejora el tiempo de arranque desde 0.
        // kA: Voltios / (RPS/s). Feedforward de aceleracion. Ayuda en la fase de spin-up.
        //     Opcional para FRC, tunear con SysId si se desea mayor precision.
        // kP = 0.5: con error de 5 RPS (300 RPM de caida) → 2.5V de correccion instantanea.
        // Bajar a 0.3 si hay oscilacion visible en las RPM. Subir a 0.8 si sigue sin recuperar.
        public static final double FLYWHEEL_P = 5.0;
        public static final double FLYWHEEL_I = 0.0;  // sin windup — re-evaluar cuando el sistema este estable
        public static final double FLYWHEEL_D = 0.0;
        public static final double FLYWHEEL_V = 0.12; 
        public static final double FLYWHEEL_S = 0.10; 
        public static final double FLYWHEEL_A = 0.0;  

        /** Tolerancia de RPM para "listo para disparar" [RPM]. */
        public static final double FLYWHEEL_RPM_TOLERANCE = 350.0;

        /** Frecuencia de actualizacion de senales del flywheel [Hz]. */
        public static final double FLYWHEEL_SIGNAL_HZ = 50.0; // Coincide con el loop de 20ms

        // --- Hood / Angulo — MotionMagicVoltage + FOC ---
        // kP: Voltios por rotacion de error. Principal termino de posicion.
        // kD: Voltios / (rot/s) de error derivativo. Reduce oscilaciones.
        // kS: Voltios para superar friccion estatica del pivot.
        // kV: Voltios / (rot/s). Feedforward de velocidad durante el movimiento.
        //     Ayuda al hood a seguir el perfil de MotionMagic sin lag.
        // kA: Voltios / (rot/s^2). Feedforward de aceleracion. Opcional.
        // kG: Voltios para sostener el hood contra la gravedad cuando esta horizontal.
        //     GravityType.Arm_Cosine: escala con cos(angulo) — maximo en horizontal, 0 en vertical.
        //     Subir kG hasta que el hood se sostenga en cualquier posicion al quitarle el PID.
        public static final double HOOD_P = 2.0;   // TODO: tunear
        public static final double HOOD_I = 0.0;
        public static final double HOOD_D = 0.0;
        /** Feedforward de gravedad para el hood [V]. Se multiplica por cos(angulo) en periodic().**/
        public static final double HOOD_GRAVITY_FF_VOLTS = 0.0;

        /** Velocidad de crucero del MotionMagic del hood [rotaciones/s]. */
        public static final double HOOD_CRUISE_VELOCITY_RPS = 20.0;
        /** Aceleracion del MotionMagic del hood [rotaciones/s^2]. */
        public static final double HOOD_ACCELERATION_RPS2   = 40.0; 

        /**
         * Gear ratio del flywheel: rotaciones del motor por rotacion de la rueda de lanzamiento.
         * Si el motor mueve la rueda directamente (sin reduccion), es 1.0.
         * Configurar en SensorToMechanismRatio para que el encoder reporte en RPS del flywheel.
         * 
         */
        public static final double FLYWHEEL_GEAR_RATIO = 1.0; 

        /**
         * Gear ratio del hood: rotaciones del motor por rotacion del mecanismo.
         * Configurado en SensorToMechanismRatio para que el encoder reporte en rotaciones del hood.
         * TODO: Medir contando dientes de la cadena o mirando el diagrama mecanico.
         */
        public static final double HOOD_GEAR_RATIO = 1.0; // TODO: medir

        /**
         * Posicion de reposo del hood = angulo mecanico minimo [grados].
         *
         * <p>El hood regresa automaticamente aqui al finalizar cada disparo.
         * Igual a HOOD_MIN_ANGLE_DEGREES — es el mismo angulo, solo con nombre semantico
         * diferente para dejar claro el proposito en el codigo de los comandos.</p>
         */
        public static final double HOOD_HOME_ANGLE_DEGREES = HOOD_MIN_ANGLE_DEGREES;

        /** Tolerancia de angulo del hood para "en posicion" [grados]. */
        public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 2.0;

        /** Frecuencia de actualizacion de senales del hood [Hz]. */
        public static final double HOOD_SIGNAL_HZ = 50.0;

        // --- Indexer ---
        // --- Indexer — control de torque (TorqueCurrentFOC) ---
        /**
         * Corriente de torque objetivo del indexer en operacion normal [A].
         * TorqueCurrentFOC mantiene este torque aunque la carga aumente (pelota entrando).
         */
        public static final double INDEXER_RUN_AMPS = 40.0;

        /**
         * Limite maximo de corriente de torque hacia adelante del indexer [A].
         * Configurado en TorqueCurrentConfigs — proteccion hardware.
         */
        public static final double INDEXER_PEAK_FORWARD_AMPS = 50.0;

        /**
         * Limite maximo de corriente de torque hacia atras del indexer [A].
         * Valor negativo — Phoenix 6 lo requiere negativo para TorqueCurrentConfigs.
         * Usado para desatascar pelotas en reversa.
         */
        public static final double INDEXER_PEAK_REVERSE_AMPS = -20.0;

        // --- Auto-desatasco del indexer ---
        // El indexer detecta un atasco cuando la velocidad cae a cero mientras
        // se comanda torque positivo. Al detectarlo, aplica corriente inversa
        // brevemente para liberar la pelota atascada y luego retoma la direccion normal.

        /**
         * Umbral de velocidad por debajo del cual se considera que el indexer esta atascado [RPS].
         * Si la velocidad del indexer cae por debajo de este valor mientras esta activo,
         * y permanece asi durante INDEXER_UNJAM_TRIGGER_SECONDS, se activa el desatasco.
         * TODO: calibrar segun la velocidad real de operacion del indexer.
         */
        public static final double INDEXER_JAM_VELOCITY_THRESHOLD_RPS = 2.0;

        /**
         * Tiempo minimo que el indexer debe estar a velocidad cero para confirmar atasco [s].
         * Evita falsos positivos durante el arranque (la inercia no es instantanea).
         * TODO: ajustar si hay falsos positivos en arranque.
         */
        public static final double INDEXER_UNJAM_TRIGGER_SECONDS = 0.12;

        /**
         * Corriente de torque aplicada al indexer durante la reversa de desatasco [A].
         * Valor negativo — opuesto a la direccion normal.
         * TODO: ajustar para que sea suficiente para liberar sin dañar mecanismo.
         */
        public static final double INDEXER_UNJAM_REVERSE_AMPS = -15.0;

        /**
         * Duracion de la reversa de desatasco del indexer [s].
         * Tras este tiempo, el indexer retoma la direccion normal automaticamente.
         * TODO: ajustar segun geometria del mecanismo.
         */
        public static final double INDEXER_UNJAM_REVERSE_SECONDS = 0.20;

        // --- Tiro por defecto (sin vision / sin tag visible) ---
        /** RPM del flywheel en modo default (hood en home). TODO: calibrar en pista. */
        public static final double DEFAULT_SHOT_RPM = 3000.0; // TODO

        /**
         * RPM de fallback para modo FIXED_HOOD cuando la odometria aun no ha sido corregida
         * por vision y la pose calculada esta fuera del rango de la lookup table [RPM].
         * La tabla cubre 0.6m–8.0m; a 1.5m efectivo → ~1220 RPM con COMPENSATION_MULTIPLICATOR.
         * Ajustar segun distancia tipica de prueba.
         */
        public static final double FIXED_HOOD_DEFAULT_RPM = 1500.0;

        // --- Limites de salida en control manual ---
        // Se usan en los comandos ManualXxxCommand para que el operador no pueda
        // pasar de un porcentaje seguro al caracterizar o probar motores individualmente.

        /** Salida maxima de cada flywheel en control manual [0.0 a 1.0]. */
        public static final double FLYWHEEL_MANUAL_MAX_OUTPUT = 0.5;

        /**
         * Salida maxima del hood en control manual [0.0 a 1.0].
         * Valor conservador porque el hood tiene un rango fisico limitado.
         */
        public static final double HOOD_MANUAL_MAX_OUTPUT = 0.3; // TODO: subir tras pruebas

        /**
         * Corriente de torque maxima del indexer en control manual [A].
         * Valor conservador para pruebas — el operador no puede exceder este limite.
         * TODO: ajustar tras pruebas.
         */
        public static final double INDEXER_MANUAL_MAX_AMPS = 40.0;  // conservador para diagnostico inicial
    }

    // =========================================================================
    // Intake (Mecanismo de recoleccion)
    // =========================================================================

    /**
     * Constantes del subsistema de intake.
     *
     * Roller: Kraken X60 (TalonFX, ID 18) — TorqueCurrentFOC (Phoenix Pro).
     * Pivot:  SparkMax + NEO v1.1 (ID 19) — PID de posicion onboard.
     */
    public static final class IntakeConstants {

        // --- Roller (Kraken X60) ---
        public static final int INTAKE_MOTOR_ID = 14;  // CAN ID fisico verificado con template mty_final
        public static final boolean INTAKE_INVERTED = false;
        public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final double INTAKE_STATOR_CURRENT_LIMIT = 40.0;

        // --- Control del roller (DutyCycleOut — ciclo de trabajo simple) ---
        // Con INTAKE_INVERTED = false (CounterClockwise_Positive):
        //   NEGATIVO = giro CW fisico = absorber pelotas   ← intake
        //   POSITIVO = giro CCW fisico = expulsar pelotas  ← outtake
        //
        // Ajuste de potencia: |INTAKE_ROLLER_DUTY_CYCLE|. Subir si no agarra bien las pelotas.
        // Rango util: 0.4 - 0.8. Empezar conservador.
        /** Ciclo de trabajo para absorcion automatica. Negativo = direccion de intake. */
        public static final double INTAKE_ROLLER_DUTY_CYCLE = -0.5;

        /** Maximo ciclo de trabajo para modo manual [0.0 - 1.0].
         *  Trigger izquierdo = absorber (negativo). Bumper izquierdo = expulsar (positivo). */
        public static final double ROLLER_MANUAL_MAX_DUTY   = 0.5;

        // TODO (futuro): implementar TorqueCurrentFOC con auto-desatasco
        // cuando se confirme licencia Phoenix Pro y se calibren los limites de corriente.

        /** Frecuencia de actualizacion de senales del roller [Hz]. */
        public static final double INTAKE_SIGNAL_HZ = 50.0;

        // --- Pivot (SparkMax + NEO v1.1) ---
        public static final int INTAKE_PIVOT_MOTOR_ID = 19;
        public static final boolean INTAKE_PIVOT_INVERTED = true;  // verificado con template mty_final
        public static final IdleMode INTAKE_PIVOT_IDLE_MODE = IdleMode.kBrake;
        public static final int INTAKE_PIVOT_CURRENT_LIMIT = 30;

        /** Rotaciones del motor por rotacion del mecanismo.  */
        public static final double PIVOT_GEAR_RATIO = 1.0;

        /** Posicion retractada del pivot [rotaciones del mecanismo]. */
        public static final double PIVOT_RETRACTED_POSITION_ROTATIONS = 0.0;

        /** Posicion desplegada del pivot [rotaciones del mecanismo].  */
        public static final double PIVOT_DEPLOYED_POSITION_ROTATIONS = -46.3;

        /** Tolerancia de posicion del pivot [rotaciones]. */
        public static final double PIVOT_POSITION_TOLERANCE_ROTATIONS = 0.5;

        // PID onboard del SparkMax 
        public static final double PIVOT_P = 0.012;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.002;

        /**
         * Salida minima del PID del pivot [porcentaje, -1.0 a 0.0].
         * Limita el voltaje de retroceso del motor del pivot.
         */
        public static final double PIVOT_MIN_OUTPUT = -1.0;

        /**
         * Salida maxima del PID del pivot [porcentaje, 0.0 a 1.0].
         * Limita el voltaje de avance del motor del pivot.
         */
        public static final double PIVOT_MAX_OUTPUT = 1.0;

        // --- Retraccion progresiva durante disparo ---
        // Activada cuando el indexer esta corriendo (ShootCommand).
        // Cada ciclo: sube STEP_UP rotaciones y espera UP_SECONDS,
        //             luego baja STEP_DOWN rotaciones y espera DOWN_SECONDS.
        // Net por ciclo = STEP_UP - STEP_DOWN → el pivot se retrae gradualmente
        // hasta llegar a PIVOT_RETRACTED_POSITION_ROTATIONS.

        /**
         * Rotaciones de retraccion por fase UP de cada ciclo.
         
         */
        public static final double PIVOT_RATCHET_STEP_UP_ROTATIONS   = 1.0;

        /**
         * Rotaciones de bajada por fase DOWN de cada ciclo.
         * Debe ser menor que PIVOT_RATCHET_STEP_UP_ROTATIONS para garantizar progreso.
  
         */
        public static final double PIVOT_RATCHET_STEP_DOWN_ROTATIONS = 0.6;

        /**
         * Tiempo que el pivot permanece en la posicion alta antes de bajar un poco [s].
 
         */
        public static final double PIVOT_RATCHET_UP_SECONDS   = 0.40;

        /**
         * Tiempo que el pivot permanece en la posicion baja antes de subir de nuevo [s].
  
         */
        public static final double PIVOT_RATCHET_DOWN_SECONDS = 0.30;

        // --- Compensacion de gravedad ---
        // El intake es pesado: la gravedad jala el brazo hacia la posicion desplegada.
        // Se aplica un feedforward positivo constante via ArbFFUnits.kVoltage en
        // setReference() del SparkMax para contrarrestar ese peso en todo momento.

        /**
         * Voltaje de feedforward positivo (hacia arriba) aplicado constantemente
         * para compensar la gravedad del brazo del intake [V].
         * Aumentar si el pivot cae lentamente; reducir si sobrepasa el setpoint.
         */
        public static final double PIVOT_GRAVITY_FF_VOLTS = 0.5;

        // --- Limites de salida en control manual ---

        // ROLLER_MANUAL_MAX_AMPS eliminado — reemplazado por ROLLER_MANUAL_MAX_DUTY en IntakeConstants

        /**
         * Salida maxima del pivot en control manual [0.0 a 1.0].
         * Valor conservador porque el pivot tiene un rango fisico limitado.
         */
        public static final double PIVOT_MANUAL_MAX_OUTPUT = 0.3; 
    }

    // =========================================================================
    // Vision (PhotonVision — 4 camaras, 2 coprocesadores)
    // =========================================================================

    /**
     * Constantes del sistema de vision con PhotonVision 2026.
     *
     * <h2>Arquitectura</h2>
     * <ul>
     *   <li>2 coprocesadores, 2 camaras cada uno.</li>
     *   <li>Cada camara corre en hilo daemon independiente.</li>
     *   <li>Los nombres de camara deben coincidir con los de la UI de PhotonVision.</li>
     *   <li>Los Transform3d deben medirse con cinta en el robot fisico.</li>
     * </ul>
     */
    public static final class VisionConstants {

        // --- IPs de los coprocesadores (agrupados por lado fisico del robot) ---
        public static final String COPROCESSOR_1_IP = "10.63.48.11"; // Lado DERECHO
        public static final String COPROCESSOR_2_IP = "10.63.48.12"; // Lado IZQUIERDO

        // --- Nombres de las camaras (exactos, incluyendo mayusculas) ---
        // Verificar que coincidan con los nombres en la UI de PhotonVision:
        //   http://10.63.48.11:5800  (derecho)  → CAM_A1_NAME, CAM_B1_NAME
        //   http://10.63.48.12:5800  (izquierdo) → CAM_A2_NAME, CAM_B2_NAME
        public static final String CAM_A1_NAME = "FRONT_RIGHT_CAM"; // Coproc 1 (derecho)
        public static final String CAM_A2_NAME = "FRONT_LEFT_CAM";  // Coproc 2 (izquierdo)
        public static final String CAM_B1_NAME = "RIGHT_CAM";       // Coproc 1 (derecho)
        public static final String CAM_B2_NAME = "LEFT_CAM";        // Coproc 2 (izquierdo)
        
        // --- Transforms robot → camara ---
        // Transform3d(Translation3d(adelante_m, izq_m, arriba_m), Rotation3d(roll, pitch, yaw))
        public static final Transform3d ROBOT_TO_CAM_A1 = new Transform3d(
            new Translation3d(0.2794,  0.2794, 0.34),
            new Rotation3d(0.0, 0, 0.0)
        );
        public static final Transform3d ROBOT_TO_CAM_A2 = new Transform3d(
            new Translation3d(-0.2794, 0.2794, 0.34),
            new Rotation3d(0.0, 0, 0.0)
        );
        public static final Transform3d ROBOT_TO_CAM_B1 = new Transform3d(
            new Translation3d(0.3429,  0.26035, 0.353),
            new Rotation3d(0.0, 0, Math.toRadians(180.0))
        );
        public static final Transform3d ROBOT_TO_CAM_B2 = new Transform3d(
            new Translation3d(-0.3429, 0.26035, 0.353),
            new Rotation3d(0.0, 0, Math.toRadians(180.0))
        );

        // =========================================================================
        // Std devs fijos por camara — ajustar manualmente segun calibracion
        // =========================================================================
        // SEMANTICA: [x_metros, y_metros, theta_radianes]
        //   MAS ALTO = MENOS confianza  (el filtro corrige menos la odometria)
        //   MAS BAJO = MAS confianza    (el filtro corrige mas la odometria)
        //
        // Para desactivar una camara completamente: pon STD_XY en 9999.0
        //
        // POR QUE STD_THETA = 9999 EN TODAS:
        // El heading lo maneja el giroscopio (Pigeon2), que es ~100x mas preciso
        // que vision. Si vision modifica theta (4 camaras x 50 Hz), pelea con el
        // gyro y el "frente" del field-centric cambia constantemente. 9999 = el
        // filtro de Kalman ignora el theta de vision por completo.
        // El unico knob util para calibrar es STD_XY, uno por camara.

        // FRONT_RIGHT_CAM
        public static final double CAM_A1_STD_XY    = 0.8;
        public static final double CAM_A1_STD_THETA = 9999.0;

        // RIGHT_CAM
        public static final double CAM_B1_STD_XY    = 0.8;
        public static final double CAM_B1_STD_THETA = 9999.0;

        // FRONT_LEFT_CAM
        public static final double CAM_A2_STD_XY    = 0.8;
        public static final double CAM_A2_STD_THETA = 9999.0;

        // LEFT_CAM
        public static final double CAM_B2_STD_XY    = 0.8;
        public static final double CAM_B2_STD_THETA = 9999.0;

        // =========================================================================
        // Filtros de calidad y rechazo de outliers
        // =========================================================================
        /** Rechazar estimaciones con ambiguedad mayor a este valor [0.0–1.0]. */
        public static final double MAX_POSE_AMBIGUITY = 0.2;

        /** Rechazar tags a mas de esta distancia [m]. */
        public static final double MAX_TAG_DISTANCE_METERS = 6.0;

        /** Si la pose de vision difiere mas de esto de la odometria actual, se rechaza.
         *  Protege contra glitches de deteccion de tags que jalarian al robot de golpe. [m] */
        public static final double MAX_VISION_POSE_JUMP_METERS = 5.0;

        /**
         * Velocidad maxima del robot para aceptar correcciones de vision [m/s].
         *
         * Por que existe esto:
         *   Cuando el robot se mueve rapido, el filtro de Kalman hace "replay" de odometria
         *   cada vez que vision manda una correccion (4 camaras x 50Hz = ~200/s). Odometria
         *   y vision jalan en direcciones opuestas y la oscilacion se acumula como distancia
         *   extra. A alta velocidad ademas la vision es menos precisa (latencia, motion blur).
         *
         *   Con 1.5 m/s: vision corrige cuando el robot esta casi parado o en movimiento lento
         *   (por ejemplo al apuntar para disparar). En movimiento rapido la odometria manda.
         *
         *   Subir si el robot corrige pose durante el auto; bajar si siguen los saltos.
         */
        public static final double VISION_MAX_SPEED_MPS = 1.5;
    }

    // =========================================================================
    // Hopper (Almacenamiento y transporte de pelotas)
    // =========================================================================

    /**
     * Constantes del subsistema de hopper.
     * 1x SparkMax + NEO v1.1 que transporta pelotas del intake al indexer.
     */
    public static final class HopperConstants {
        // 1x TalonFX (Kraken X60) — transporta pelotas del intake al indexer.
        // Siempre corre junto al indexer.
        public static final int              HOPPER_MOTOR_ID          = 27;                     // CAN ID verificado con template mty_final
        public static final boolean          HOPPER_INVERTED          = true;                   // CW_Positive — verificado con template
        public static final NeutralModeValue HOPPER_NEUTRAL_MODE      = NeutralModeValue.Coast;
        public static final double           HOPPER_DUTY_CYCLE        = 0.6;                    // velocidad normal de transporte [0-1]
        public static final double           HOPPER_MANUAL_MAX_OUTPUT = 0.6;                    // maximo en control manual [0-1]

        private HopperConstants() {}
    }
}

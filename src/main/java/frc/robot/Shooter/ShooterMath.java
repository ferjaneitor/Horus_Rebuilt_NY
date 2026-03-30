package frc.robot.Shooter;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Clase utilitaria de matematica para el shooter parabolico — v3 (optimizada).
 *
 * <h2>Principio de diseno: tiempo de vuelo constante</h2>
 * <p>Con el apex fijo, T = t_rise + t_fall depende SOLO de las alturas
 * (shooter, apex, objetivo), no de la distancia. Por eso T_TOTAL_SECONDS
 * es una constante pre-calculada. A tiempo de ejecucion solo se necesitan:</p>
 * <ol>
 *   <li>La distancia efectiva al objetivo compensado.</li>
 *   <li>{@code vx = d / T_TOTAL_SECONDS}</li>
 *   <li>{@code V  = hypot(vx, VY0_METERS_PER_SECOND)}</li>
 *   <li>{@code RPM = V * RPM_PER_MS}</li>
 *   <li>{@code theta = atan2(VY0, vx)}</li>
 * </ol>
 * <p>Todas las raices cuadradas y multiplicaciones costosas ya estan en
 * {@link ShooterConstants} como {@code public static final double}.</p>
 *
 * <h2>Seleccion automatica de objetivo por zona</h2>
 * <p>El campo se divide en tres zonas segun la posicion X del robot:</p>
 * <ul>
 *   <li><b>Zona propia</b>: x &lt; RED_ZONE_MAX_X (rojo) o x &gt; BLUE_ZONE_MIN_X (azul) →
 *       objetivo SCORING.</li>
 *   <li><b>Zona neutral o enemiga</b>: objetivo PASSING mas cercano en Y al robot.</li>
 * </ul>
 *
 * <h2>Compensacion de movimiento del robot (traslacion XY)</h2>
 * <pre>
 *   fieldSpeeds  = fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation())
 *   target_adj   = targetXY - fieldSpeeds_xy * T_TOTAL_SECONDS
 *   d_efectiva   = |robot - target_adj| - SHOOTER_X_OFFSET
 * </pre>
 *
 * <h2>Uso tipico</h2>
 * <pre>{@code
 * ShooterMath.ShooterResult r = ShooterMath.calculateAutoTarget(
 *     drivetrain.getState().Pose,
 *     drivetrain.getState().Speeds,
 *     DriverStation.getAlliance().orElse(Alliance.Blue)
 * );
 * ShooterMath.publishToSmartDashboard(r);
 * if (r.isValidShot()) {
 *     flywheel.setRPM(r.flywheelRPM());
 *     pivot.setAngle(r.shooterAngleDegrees());
 *     drivetrain.setTargetHeading(r.robotHeadingDegrees());
 * }
 * }</pre>
 */
public final class ShooterMath {

    private ShooterMath() {
        throw new UnsupportedOperationException("ShooterMath es una clase utilitaria, no debe instanciarse.");
    }

    // =========================================================================
    // Modo de shooter — seleccionable desde SmartDashboard
    // =========================================================================

    /**
     * Modo de operacion del shooter.
     * <ul>
     *   <li>{@code VARIABLE_HOOD} — hood ajusta angulo + flywheel ajusta RPM segun distancia
     *       (matematica parabolica completa, auto-aim activo). <b>Modo por defecto.</b></li>
     *   <li>{@code FIXED_HOOD} — hood queda en posicion home, solo el flywheel ajusta RPM
     *       segun lookup table de distancia. Sin auto-aim ni matematica parabolica.</li>
     * </ul>
     */
    public enum ShooterMode { VARIABLE_HOOD, FIXED_HOOD }

    // Lookup table: distancia efectiva (m) → RPM del flywheel para hood fijo.
    // El hood se queda en HOOD_HOME_ANGLE_DEGREES en todo momento.
    private static final TreeMap<Double, Double> FIXED_HOOD_RPM_TABLE = new TreeMap<>();
    static {
        FIXED_HOOD_RPM_TABLE.put(0.6, 112.787*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(0.8, 150.383*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.0, 187.978*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.2, 1039.278*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.4, 1061.717*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.6, 1092.640*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.8, 1127.238*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.0, 1163.383*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.2, 1200.032*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.4, 1236.641*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.6, 1272.917*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.8, 1308.702*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.0, 1343.913*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.2, 1378.510*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.4, 1412.480*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.6, 1445.825*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.8, 1478.556*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.0, 1510.690*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.2, 1542.245*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.4, 1573.244*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.6, 1603.707*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.8, 1633.655*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.0, 1663.109*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.2, 1692.089*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.4, 1720.615*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.6, 1748.704*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.8, 1776.375*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.0, 1803.643*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.2, 1830.525*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.4, 1857.035*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.6, 1883.187*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.8, 1908.994*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.0, 1934.470*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.2, 1959.626*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.4, 1984.472*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.6, 2009.020*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.8, 2033.280*ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(8.0, 2057.262*ShooterConstants.COMPENSATION_MULTIPLICATOR);
    }

    // =========================================================================
    // Tipos publicos
    // =========================================================================

    /**
     * Tipo de objetivo de disparo.
     * <ul>
     *   <li>{@code SCORING} — objetivo principal de anotacion (speaker / embudo)</li>
     *   <li>{@code PASS_1}  — passing target lado sur (Y bajo)</li>
     *   <li>{@code PASS_2}  — passing target lado norte (Y alto)</li>
     * </ul>
     */
    public enum TargetType { SCORING, PASS_1, PASS_2 }

    /**
     * Zona del campo donde se encuentra el robot.
     * <ul>
     *   <li>{@code RED_ALLIANCE}  — zona de la alianza roja (x &lt; RED_ZONE_MAX_X)</li>
     *   <li>{@code NEUTRAL}       — zona neutral (entre los dos scoring targets en X)</li>
     *   <li>{@code BLUE_ALLIANCE} — zona de la alianza azul (x &gt; BLUE_ZONE_MIN_X)</li>
     * </ul>
     */
    public enum FieldZone { RED_ALLIANCE, NEUTRAL, BLUE_ALLIANCE }

    /**
     * Zona de disparo: posicion 3D del objetivo en coordenadas del campo.
     *
     * @param targetPosition {@code (x, y)} = posicion en el campo; {@code z} = altura objetivo.
     * @param type           Tipo de objetivo.
     * @param displayName    Nombre para SmartDashboard y logs.
     */
    public record ShootingZone(
        Translation3d targetPosition,
        TargetType    type,
        String        displayName
    ) {}

    /**
     * Resultado completo del calculo de tiro.
     *
     * <p>Si {@code isValidShot} es {@code false}, los valores numericos siguen
     * siendo calculados para debug — solo {@code invalidReason} describe el problema.</p>
     *
     * <p><b>Nota sobre angulos</b>: {@code shooterAngleDegrees} es el angulo MECANICO
     * del hood (lo que se envia al motor). El angulo de SALIDA de la pelota es su
     * complementario: {@code exitAngleDegrees = 90° - shooterAngleDegrees}.</p>
     *
     * @param flywheelVelocityMetersPerSecond Velocidad de salida de la pelota [m/s].
     * @param flywheelRPM                     RPM requeridas del flywheel.
     * @param shooterAngleDegrees             Angulo MECANICO del hood [grados].
     *                                        Rango: [HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES].
     *                                        Angulo de salida = 90° - este valor.
     * @param robotHeadingDegrees             Heading que debe tener el robot [-180, 180] grados.
     * @param distanceMeters                  Distancia horizontal robot-objetivo compensado [m].
     * @param timeOfFlightSeconds             Tiempo de vuelo = {@code T_TOTAL_SECONDS} [s].
     * @param isValidShot                     {@code true} si todos los limites se cumplen.
     * @param invalidReason                   Descripcion del fallo (cadena vacia si valido).
     */
    public record ShooterResult(
        double  flywheelVelocityMetersPerSecond,
        double  flywheelRPM,
        double  shooterAngleDegrees,    // angulo MECANICO del hood (no el de salida)
        double  robotHeadingDegrees,
        double  distanceMeters,
        double  timeOfFlightSeconds,
        boolean isValidShot,
        String  invalidReason
    ) {
        /** Resultado invalido con valores numericos a 0 y una razon. */
        public static ShooterResult invalid(String reason) {
            return new ShooterResult(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, reason);
        }

        /**
         * Angulo de SALIDA de la pelota desde la horizontal [grados].
         * Es el angulo complementario del angulo mecanico del hood.
         *
         * @return angulo de salida = 90° - shooterAngleDegrees.
         */
        public double exitAngleDegrees() {
            return ShooterConstants.COMPLEMENTARY_ANGLE_DEGREES - shooterAngleDegrees;
        }
    }

    // =========================================================================
    // Deteccion de zona del campo
    // =========================================================================

    /**
     * Determina en que zona del campo se encuentra el robot segun su posicion X.
     *
     * <p>Las zonas se definen a partir de los scoring targets del campo activo
     * ({@link FieldConstants#ACTIVE_FIELD_TYPE}):</p>
     * <ul>
     *   <li>x &lt; {@code RED_ZONE_MAX_X}  → RED_ALLIANCE</li>
     *   <li>x &gt; {@code BLUE_ZONE_MIN_X} → BLUE_ALLIANCE</li>
     *   <li>En medio                         → NEUTRAL</li>
     * </ul>
     *
     * @param robotPose Pose actual del robot.
     * @return {@link FieldZone} donde esta el robot.
     */
    public static FieldZone detectZone(Pose2d robotPose) {
        double x = robotPose.getX();
        if (x < FieldConstants.RED_ZONE_MAX_X)  return FieldZone.RED_ALLIANCE;
        if (x > FieldConstants.BLUE_ZONE_MIN_X) return FieldZone.BLUE_ALLIANCE;
        return FieldZone.NEUTRAL;
    }

    // =========================================================================
    // Zonas de disparo
    // =========================================================================

    /**
     * Retorna la {@link ShootingZone} para el tipo de objetivo y alianza dados.
     *
     * @param type     Tipo de objetivo.
     * @param alliance Alianza del robot.
     * @return Zona de disparo configurada con las coordenadas de {@link ShooterConstants}.
     */
    public static ShootingZone getZone(TargetType type, Alliance alliance) {
        Translation3d position;
        String        name;

        if (alliance == Alliance.Blue) {
            switch (type) {
                case SCORING -> { position = ShooterConstants.BLUE_SCORING_TARGET; name = "Blue Scoring"; }
                case PASS_1  -> { position = ShooterConstants.BLUE_PASS_TARGET_1;  name = "Blue Pass 1";  }
                case PASS_2  -> { position = ShooterConstants.BLUE_PASS_TARGET_2;  name = "Blue Pass 2";  }
                default -> throw new IllegalArgumentException("[ShooterMath] TargetType desconocido: " + type);
            }
        } else {
            switch (type) {
                case SCORING -> { position = ShooterConstants.RED_SCORING_TARGET; name = "Red Scoring"; }
                case PASS_1  -> { position = ShooterConstants.RED_PASS_TARGET_1;  name = "Red Pass 1";  }
                case PASS_2  -> { position = ShooterConstants.RED_PASS_TARGET_2;  name = "Red Pass 2";  }
                default -> throw new IllegalArgumentException("[ShooterMath] TargetType desconocido: " + type);
            }
        }

        return new ShootingZone(position, type, name);
    }

    // =========================================================================
    // Calculo con seleccion automatica de objetivo (API principal)
    // =========================================================================

    /**
     * Calcula los parametros de disparo seleccionando automaticamente el objetivo
     * segun la zona del campo donde se encuentra el robot y su alianza.
     *
     * <h3>Logica de seleccion</h3>
     * <ul>
     *   <li>En zona propia (Red en RED_ALLIANCE, Blue en BLUE_ALLIANCE): objetivo SCORING.</li>
     *   <li>En zona neutral o enemiga: objetivo PASSING, eligiendo PASS_1 o PASS_2
     *       segun cual sea mas cercano al Y actual del robot.</li>
     * </ul>
     *
     * @param robotPose   Pose actual del robot.
     * @param robotSpeeds Velocidades actuales del robot (robot-relativas).
     * @param alliance    Alianza actual del robot.
     * @return {@link ShooterResult} completo con flag de validez.
     */
    public static ShooterResult calculateAutoTarget(
        Pose2d        robotPose,
        ChassisSpeeds robotSpeeds,
        Alliance      alliance
    ) {
        FieldZone zone = detectZone(robotPose);

        boolean inOwnZone = (alliance == Alliance.Blue && zone == FieldZone.BLUE_ALLIANCE)
                         || (alliance == Alliance.Red  && zone == FieldZone.RED_ALLIANCE);

        if (inOwnZone) {
            return calculate(robotPose, robotSpeeds, TargetType.SCORING, alliance);
        }

        // Zona neutral o enemiga: elegir PASS_1 (sur) o PASS_2 (norte) segun proximidad en Y
        double distToPass1 = Math.abs(robotPose.getY() - FieldConstants.PASS_Y_SOUTH);
        double distToPass2 = Math.abs(robotPose.getY() - FieldConstants.PASS_Y_NORTH);
        TargetType passTarget = (distToPass1 <= distToPass2) ? TargetType.PASS_1 : TargetType.PASS_2;

        return calculate(robotPose, robotSpeeds, passTarget, alliance);
    }

    // =========================================================================
    // Calculo principal
    // =========================================================================

    /**
     * Calcula los parametros de disparo hacia la zona objetivo,
     * compensando el movimiento del robot en tiempo real.
     *
     * <p>Gracias a que {@code T_TOTAL_SECONDS} es pre-calculado como constante,
     * el calculo en este metodo se reduce a:</p>
     * <ol>
     *   <li>Convertir speeds robot-relativas a field-relativas.</li>
     *   <li>Desplazar el objetivo virtual: {@code target_adj = target - vel * T}</li>
     *   <li>Calcular distancia efectiva al objetivo ajustado.</li>
     *   <li>{@code vx = d / T},  {@code V = hypot(vx, VY0)},
     *       {@code RPM = V * RPM_PER_MS},  {@code theta = atan2(VY0, vx)}</li>
     *   <li>Comparar distancia efectiva contra limites pre-calculados.</li>
     * </ol>
     *
     * @param robotPose   Pose actual del robot (de la odometria).
     * @param robotSpeeds Velocidades actuales del robot (robot-relativas, de Phoenix 6).
     * @param targetZone  Zona de disparo seleccionada.
     * @return {@link ShooterResult} completo con flag de validez.
     */
    public static ShooterResult calculate(
        Pose2d        robotPose,
        ChassisSpeeds robotSpeeds,
        ShootingZone  targetZone
    ) {
        // --- Extraer posiciones 2D ---
        Translation2d robotXY  = robotPose.getTranslation();
        Translation3d target3d = targetZone.targetPosition();
        Translation2d targetXY = new Translation2d(target3d.getX(), target3d.getY());

        // --- Compensacion de movimiento ---
        // T_TOTAL_SECONDS es constante → la compensacion es exacta en una sola pasada.
        // Convertir speeds robot-relativas a field-relativas para operar en coordenadas del campo.
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, robotPose.getRotation()
        );

        double adjTargetX = targetXY.getX() - fieldSpeeds.vxMetersPerSecond * ShooterConstants.T_TOTAL_SECONDS;
        double adjTargetY = targetXY.getY() - fieldSpeeds.vyMetersPerSecond * ShooterConstants.T_TOTAL_SECONDS;
        Translation2d adjTargetXY = new Translation2d(adjTargetX, adjTargetY);

        // --- Distancia efectiva (horizontal, shooter → objetivo compensado) ---
        double totalDistance = robotXY.getDistance(adjTargetXY);
        double d             = totalDistance - ShooterConstants.SHOOTER_X_OFFSET_METERS;

        if (d <= 0.0) {
            return ShooterResult.invalid(String.format(
                "Distancia efectiva d=%.2f m invalida. El robot debe estar a mas de %.2f m del objetivo.",
                d, ShooterConstants.SHOOTER_X_OFFSET_METERS + 0.05
            ));
        }

        // --- Parametros del tiro (con constantes pre-calculadas) ---
        // VY0_METERS_PER_SECOND y T_TOTAL_SECONDS son constantes: sin sqrt ni divisiones extra.
        double vx       = d / ShooterConstants.T_TOTAL_SECONDS;
        double velocity = Math.hypot(vx, ShooterConstants.VY0_METERS_PER_SECOND);
        double rpm      = velocity * ShooterConstants.RPM_PER_MS;

        // --- Conversion de angulo de salida a angulo mecanico del hood ---
        // exitAngleDeg: angulo desde la horizontal de la trayectoria de la pelota (fisica).
        // hoodAngleDeg: angulo mecanico que debe tener el hood (lo que se envia al motor).
        //
        // Relacion complementaria: hoodAngle = 90° - exitAngle
        // Si la pelota debe salir a 30°, el hood debe estar a 60° mecanicos.
        double exitAngleDeg = Math.toDegrees(Math.atan2(ShooterConstants.VY0_METERS_PER_SECOND, vx));
        double hoodAngleDeg = ShooterConstants.COMPLEMENTARY_ANGLE_DEGREES - exitAngleDeg;

        // Aplicar limites fisicos del mecanismo como capa de seguridad adicional.
        // Los limites de distancia (MIN/MAX_EFFECTIVE_DIST_BY_ANGLE) ya garantizan que
        // el angulo no exceda los rangos, pero se aplica clamp como proteccion extra.
        hoodAngleDeg = Math.max(ShooterConstants.HOOD_MIN_ANGLE_DEGREES,
                       Math.min(ShooterConstants.HOOD_MAX_ANGLE_DEGREES, hoodAngleDeg));

        // --- Heading del robot hacia el objetivo compensado ---
        Translation2d toTarget = adjTargetXY.minus(robotXY);
        double headingDeg = new Rotation2d(
            Math.atan2(toTarget.getY(), toTarget.getX())
        ).getDegrees(); // [-180, 180]

        // --- Validacion con limites pre-calculados ---
        // Los limites de angulo y RPM ya fueron convertidos a limites de distancia efectiva
        // en ShooterConstants, evitando recalcular las funciones trigonometricas aqui.

        if (d < ShooterConstants.MIN_EFFECTIVE_DIST_BY_ANGLE_METERS) {
            return new ShooterResult(velocity, rpm, hoodAngleDeg, headingDeg, totalDistance,
                ShooterConstants.T_TOTAL_SECONDS, false,
                String.format(
                    "Robot demasiado cerca. d=%.2f m < min=%.2f m "
                    + "(salida superaria %.0f deg → hood en %.0f deg).",
                    d, ShooterConstants.MIN_EFFECTIVE_DIST_BY_ANGLE_METERS,
                    ShooterConstants.MAX_SHOT_ANGLE_DEGREES,
                    ShooterConstants.HOOD_MIN_ANGLE_DEGREES));
        }

        if (d > ShooterConstants.MAX_EFFECTIVE_DIST_BY_ANGLE_METERS) {
            return new ShooterResult(velocity, rpm, hoodAngleDeg, headingDeg, totalDistance,
                ShooterConstants.T_TOTAL_SECONDS, false,
                String.format(
                    "Robot demasiado lejos. d=%.2f m > max=%.2f m "
                    + "(salida < %.0f deg → hood en %.0f deg).",
                    d, ShooterConstants.MAX_EFFECTIVE_DIST_BY_ANGLE_METERS,
                    ShooterConstants.MIN_SHOT_ANGLE_DEGREES,
                    ShooterConstants.HOOD_MAX_ANGLE_DEGREES));
        }

        if (d > ShooterConstants.MAX_EFFECTIVE_DIST_BY_RPM_METERS) {
            return new ShooterResult(velocity, rpm, hoodAngleDeg, headingDeg, totalDistance,
                ShooterConstants.T_TOTAL_SECONDS, false,
                String.format(
                    "Distancia excede limite de RPM. d=%.2f m > max_rpm=%.2f m (RPM=%.0f > %.0f).",
                    d, ShooterConstants.MAX_EFFECTIVE_DIST_BY_RPM_METERS,
                    rpm, ShooterConstants.MAX_FLYWHEEL_RPM));
        }

        if (totalDistance > ShooterConstants.MAX_SHOT_DISTANCE_METERS) {
            return new ShooterResult(velocity, rpm, hoodAngleDeg, headingDeg, totalDistance,
                ShooterConstants.T_TOTAL_SECONDS, false,
                String.format("Distancia total %.1f m > maximo configurado %.1f m.",
                    totalDistance, ShooterConstants.MAX_SHOT_DISTANCE_METERS));
        }

        // Tiro valido
        return new ShooterResult(velocity, rpm, hoodAngleDeg, headingDeg,
            totalDistance, ShooterConstants.T_TOTAL_SECONDS, true, "");
    }

    /**
     * Calcula los parametros de disparo usando tipo de objetivo y alianza.
     * Obtiene la zona con {@link #getZone} y delega al metodo principal.
     *
     * @param robotPose   Pose actual del robot.
     * @param robotSpeeds Velocidades actuales del robot (robot-relativas).
     * @param targetType  Tipo de objetivo.
     * @param alliance    Alianza actual del robot.
     * @return {@link ShooterResult} completo.
     */
    public static ShooterResult calculate(
        Pose2d        robotPose,
        ChassisSpeeds robotSpeeds,
        TargetType    targetType,
        Alliance      alliance
    ) {
        return calculate(robotPose, robotSpeeds, getZone(targetType, alliance));
    }

    // =========================================================================
    // SmartDashboard
    // =========================================================================

    /**
     * Publica el resultado completo en SmartDashboard bajo el prefijo {@code "Shooter/"}.
     *
     * <table>
     *   <tr><th>Clave</th><th>Valor</th></tr>
     *   <tr><td>Shooter/FlywheelRPM</td><td>RPM objetivo del flywheel</td></tr>
     *   <tr><td>Shooter/AngleDeg</td><td>Angulo del pivot [grados]</td></tr>
     *   <tr><td>Shooter/RobotHeading</td><td>Heading objetivo del robot [grados]</td></tr>
     *   <tr><td>Shooter/Distance</td><td>Distancia robot-objetivo [m]</td></tr>
     *   <tr><td>Shooter/TimeOfFlight</td><td>Tiempo de vuelo [s]</td></tr>
     *   <tr><td>Shooter/ReadyToShoot</td><td>true si el tiro es valido</td></tr>
     *   <tr><td>Shooter/InvalidReason</td><td>Razon del fallo (vacio si valido)</td></tr>
     * </table>
     *
     * @param result Resultado a publicar.
     */
    public static void publishToSmartDashboard(ShooterResult result) {
        SmartDashboard.putNumber ("Shooter/FlywheelRPM",    result.flywheelRPM());
        // HoodTargetDeg: angulo MECANICO que el hood debe alcanzar (va al motor)
        SmartDashboard.putNumber ("Shooter/HoodTargetDeg",  result.shooterAngleDegrees());
        // ExitAngleDeg: angulo de salida de la pelota desde la horizontal (fisica)
        // = 90° - HoodTargetDeg (complementario)
        SmartDashboard.putNumber ("Shooter/ExitAngleDeg",   result.exitAngleDegrees());
        SmartDashboard.putNumber ("Shooter/RobotHeading",   result.robotHeadingDegrees());
        SmartDashboard.putNumber ("Shooter/Distance",       result.distanceMeters());
        SmartDashboard.putNumber ("Shooter/TimeOfFlight",   result.timeOfFlightSeconds());
        SmartDashboard.putBoolean("Shooter/ValidShot",      result.isValidShot());
        SmartDashboard.putString ("Shooter/InvalidReason",  result.invalidReason());
    }

    // =========================================================================
    // Utilitario de conversion
    // =========================================================================

    /**
     * Convierte velocidad lineal de la pelota a RPM del flywheel.
     * Equivalente a {@code velocity * ShooterConstants.RPM_PER_MS}.
     *
     * @param velocityMetersPerSecond Velocidad de la pelota [m/s]. Debe ser &gt; 0.
     * @return RPM del flywheel.
     * @throws IllegalArgumentException Si la velocidad es 0 o negativa.
     */
    public static double velocityToRPM(double velocityMetersPerSecond) {
        if (velocityMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException(String.format(
                "[ShooterMath] velocityMetersPerSecond debe ser > 0, recibido: %.3f",
                velocityMetersPerSecond
            ));
        }
        return velocityMetersPerSecond * ShooterConstants.RPM_PER_MS;
    }

    // =========================================================================
    // Calculo para hood fijo (lookup table)
    // =========================================================================

    /**
     * Calcula los parametros de disparo en modo hood fijo.
     *
     * <p>El hood permanece en {@link ShooterConstants#HOOD_HOME_ANGLE_DEGREES}.
     * Solo el flywheel ajusta su RPM segun la distancia al objetivo de anotacion,
     * usando interpolacion lineal sobre {@code FIXED_HOOD_RPM_TABLE}.</p>
     *
     * <p>No se aplica compensacion de movimiento ni auto-aim de heading —
     * el operador apunta el robot manualmente.</p>
     *
     * @param robotPose Pose actual del robot (de la odometria).
     * @param alliance  Alianza actual del robot.
     * @return {@link ShooterResult} con {@code shooterAngleDegrees = HOOD_HOME_ANGLE_DEGREES}
     *         y RPM interpolado de la lookup table.
     */
    public static ShooterResult calculateFixedHood(Pose2d robotPose, Alliance alliance) {
        Translation3d target3d = (alliance == Alliance.Blue)
            ? ShooterConstants.BLUE_SCORING_TARGET
            : ShooterConstants.RED_SCORING_TARGET;
        Translation2d targetXY = new Translation2d(target3d.getX(), target3d.getY());

        double totalDistance = robotPose.getTranslation().getDistance(targetXY);
        double d = totalDistance - ShooterConstants.SHOOTER_X_OFFSET_METERS;

        if (d <= 0.0) {
            return ShooterResult.invalid(String.format(
                "Distancia efectiva d=%.2f m invalida (demasiado cerca del objetivo).", d));
        }

        if (totalDistance > ShooterConstants.MAX_SHOT_DISTANCE_METERS) {
            return ShooterResult.invalid(String.format(
                "Demasiado lejos: %.1f m > maximo %.1f m.",
                totalDistance, ShooterConstants.MAX_SHOT_DISTANCE_METERS));
        }

        double rpm = interpolateTable(FIXED_HOOD_RPM_TABLE, d);

        // Heading informativo hacia el objetivo (el operador decide si apuntar)
        Translation2d toTarget = targetXY.minus(robotPose.getTranslation());
        double headingDeg = new Rotation2d(
            Math.atan2(toTarget.getY(), toTarget.getX())
        ).getDegrees();

        return new ShooterResult(
            rpm / ShooterConstants.RPM_PER_MS,
            rpm,
            ShooterConstants.HOOD_HOME_ANGLE_DEGREES,
            headingDeg,
            totalDistance,
            0.0,
            true,
            ""
        );
    }

    /**
     * Interpolacion lineal sobre una lookup table ordenada por clave.
     * Si {@code x} esta fuera del rango, retorna el valor del extremo mas cercano (clamp).
     */
    private static double interpolateTable(TreeMap<Double, Double> table, double x) {
        Map.Entry<Double, Double> lo = table.floorEntry(x);
        Map.Entry<Double, Double> hi = table.ceilingEntry(x);
        if (lo == null) return hi.getValue();
        if (hi == null) return lo.getValue();
        if (lo.getKey().equals(hi.getKey())) return lo.getValue();
        double t = (x - lo.getKey()) / (hi.getKey() - lo.getKey());
        return lo.getValue() + t * (hi.getValue() - lo.getValue());
    }
}

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.Drive.TunerConstants;
import frc.robot.Drive.commands.AutoAimDriveCommand;
import frc.robot.Intake.IntakePivotSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.commands.DeployPivotCommand;
import frc.robot.Intake.commands.RetractPivotCommand;
import frc.robot.Intake.commands.RunIntakeCommand;
import frc.robot.Shooter.ShooterMath.ShooterMode;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.commands.ShootCommand;
import frc.robot.Util.Manual_Commands.ManualFlywheelsCommand;
import frc.robot.Util.Manual_Commands.ManualHoodCommand;
import frc.robot.Util.Manual_Commands.ManualIndexerCommand;
import frc.robot.Util.Manual_Commands.ManualIntakePivotCommand;
import frc.robot.Util.Manual_Commands.ManualIntakeRollerCommand;
import frc.robot.Vision.VisionSubsystem;

/**
 * Clase central de configuracion del robot.
 *
 * Aqui se declaran los subsistemas, los controles del driver/operator,
 * y se conectan los comandos a los botones.
 *
 * NO contiene logica de juego — eso va en los subsistemas y comandos.
 *
 * <h2>Mapeo de controles — Driver (controller 0)</h2>
 * <pre>
 * Joystick izquierdo      → Traslacion del robot (X/Y field-centric)
 * Joystick derecho X      → Rotacion del robot (omega)
 * Right Trigger           → Disparar (ShootCommand — vision-based o default)
 * Left Trigger            → Correr intake (mientras se mantiene presionado)
 * X                       → Auto-aim (robot apunta al objetivo automaticamente)
 * Y                       → Resetear heading field-centric
 * Right Bumper            → Deploy intake (extender el intake)
 * Left Bumper             → Retract intake (recoger el intake)
 * Back + Y/X              → SysId Dynamic forward/reverse (solo desarrollo)
 * Start + Y/X             → SysId Quasistatic forward/reverse (solo desarrollo)
 * A                       → Disponible
 * B                       → Disponible
 * </pre>
 *
 * <h2>Mapeo de controles — Test (controller 2)</h2>
 * <p>Usado exclusivamente para pruebas de mecanismos ANTES de usar el driver
 * controller. Permite ejercitar cada mecanismo individualmente a velocidad manual.</p>
 * <pre>
 * Right Trigger (eje)     → Ambos flywheels a la vez (izq=+, der=−)
 * Left Trigger  (eje)     → Indexer manual
 * Left Stick Y  (deflect) → Intake Pivot manual (arriba/abajo)
 * Right Stick Y (deflect) → Hood manual (arriba/abajo)
 * A  (mantener)           → Intake Roller → adelante
 * B  (mantener)           → Intake Roller → reversa
 * </pre>
 */
public class RobotContainer {

    // --- Velocidades de referencia para el drivetrain ---
    private final double maxSpeedMetersPerSecond   = DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    private final double maxAngularRateRadPerSecond = DriveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND;

    // --- Requests de swerve para teleop ---
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeedMetersPerSecond * DriveConstants.JOYSTICK_DEADBAND_PERCENT)
        .withRotationalDeadband(maxAngularRateRadPerSecond * DriveConstants.JOYSTICK_DEADBAND_PERCENT)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // --- Telemetria ---
    private final Telemetry logger = new Telemetry(maxSpeedMetersPerSecond);

    // --- Controles ---
    private final CommandXboxController driverController = new CommandXboxController(0);
    //private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(2);

    // --- Subsistemas ---
    public final CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
    public final VisionSubsystem         vision      = new VisionSubsystem(drivetrain);
    public final ShooterSubsystem        shooter     = new ShooterSubsystem();
    public final IntakeSubsystem         intake      = new IntakeSubsystem();
    public final IntakePivotSubsystem    intakePivot = new IntakePivotSubsystem();

    // --- Comando de auto-aim (reutilizable) ---
    private final AutoAimDriveCommand autoAimDrive = new AutoAimDriveCommand(
        drivetrain,
        () -> -driverController.getLeftY()  * maxSpeedMetersPerSecond,
        () -> -driverController.getLeftX()  * maxSpeedMetersPerSecond
    );

    // --- Modo del shooter ---
    // Seleccionable desde SmartDashboard antes de cada partido.
    // VARIABLE_HOOD (defecto): angulo + RPM calculados por ShooterMath.
    // FIXED_HOOD: hood en home, RPM por lookup table segun distancia.
    private final SendableChooser<ShooterMode> shooterModeChooser = new SendableChooser<>();

    // --- Autonomo ---
    // Selector de rutinas autonomas creadas en PathPlanner GUI.
    // Los archivos .auto van en deploy/pathplanner/autos/
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Configurar selector de modo del shooter
        shooterModeChooser.setDefaultOption("Hood Variable (Auto-Aim)", ShooterMode.VARIABLE_HOOD);
        shooterModeChooser.addOption("Hood Fijo (Lookup Table)",        ShooterMode.FIXED_HOOD);
        SmartDashboard.putData("Shooter Mode", shooterModeChooser);

        // Registrar comandos nombrados de PathPlanner ANTES de buildAutoChooser().
        // Estos nombres deben coincidir exactamente con los usados en PathPlanner GUI.
        registerPathPlannerCommands();

        // El autoChooser se construye DESPUES de que el drivetrain (y PathPlanner)
        // esten inicializados, para que AutoBuilder pueda listar los autos disponibles.
        // Si PathPlanner no se pudo configurar (falta settings.json), se crea un
        // chooser vacio para que el robot no crashee — el auto quedara desactivado.
        if (AutoBuilder.isConfigured()) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption(
                "Sin auto — abre PathPlanner GUI para generar settings.json",
                Commands.none()
            );
            DriverStation.reportWarning(
                "PathPlanner no configurado: falta deploy/pathplanner/settings.json. "
                + "Abre PathPlanner GUI y configura el robot para habilitar el auto.",
                false
            );
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    /**
     * Registra los comandos con nombre para que PathPlanner pueda invocarlos
     * desde las rutinas autonomas (archivos .auto).
     *
     * <p>Cada nombre debe coincidir exactamente (mayusculas/minusculas) con
     * el nombre configurado en PathPlanner GUI → Named Commands.</p>
     */
    private void registerPathPlannerCommands() {
        // Disparo completo — el modo (variable/fijo) se lee del chooser en tiempo de ejecucion
        NamedCommands.registerCommand("Shoot",
            new ShootCommand(shooter, vision, drivetrain, intakePivot, shooterModeChooser::getSelected));

        // Intake
        NamedCommands.registerCommand("RunIntake",
            new RunIntakeCommand(intake));
        NamedCommands.registerCommand("DeployIntake",
            new DeployPivotCommand(intakePivot));
        NamedCommands.registerCommand("RetractIntake",
            new RetractPivotCommand(intakePivot));
    }

    private void configureBindings() {

        // =====================================================================
        // Default command del drivetrain: manejo field-centric
        // =====================================================================
        // Left joystick  → traslacion (X=adelante, Y=izquierda)
        // Right joystick X → rotacion (omega)
        // Los ejes se invierten porque el joystick reporta + hacia abajo/derecha,
        // pero WPILib define + hacia adelante/izquierda.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY()  * maxSpeedMetersPerSecond)
                    .withVelocityY(-driverController.getLeftX()  * maxSpeedMetersPerSecond)
                    .withRotationalRate(-driverController.getRightX() * maxAngularRateRadPerSecond)
            )
        );

        // Freno mientras el robot esta deshabilitado
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // =====================================================================
        // Disparo
        // =====================================================================
        // Right Trigger: Disparar mientras se mantiene presionado.
        // ShootCommand determina automaticamente el objetivo segun zona y vision.
        // Si no hay tags visibles o el tiro es invalido, usa modo default (RPM fija).
        // El pivot del intake cicla automaticamente durante el disparo.
        driverController.rightTrigger().whileTrue(
            new ShootCommand(shooter, vision, drivetrain, intakePivot, shooterModeChooser::getSelected)
        );

        // =====================================================================
        // Intake
        // =====================================================================
        // Left Trigger: Correr el roller del intake mientras se mantiene presionado.
        driverController.leftTrigger().whileTrue(
            new RunIntakeCommand(intake)
        );

        // Right Bumper: Deploy (extender intake a posicion de recoleccion).
        driverController.rightBumper().onTrue(
            new DeployPivotCommand(intakePivot)
        );

        // Left Bumper: Retract (recoger intake a posicion de home).
        driverController.leftBumper().onTrue(
            new RetractPivotCommand(intakePivot)
        );

        // =====================================================================
        // Auto-aim
        // =====================================================================
        // X: Auto-aim — el robot gira automaticamente hacia el objetivo calculado
        // por ShooterMath (SCORING o PASSING segun zona).
        // El driver mantiene control de traslacion XY.
        // Al soltar, el default command retoma el control completo (incluyendo omega).
        driverController.x().whileTrue(autoAimDrive);

        // =====================================================================
        // Reset heading
        // =====================================================================
        // Y: Resetear el heading field-centric.
        // Despues de llamar seedFieldCentric, "adelante" en el campo corresponde
        // a la direccion que apunta el robot en ese momento.
        driverController.y().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        // =====================================================================
        // SysId (solo para caracterizacion en desarrollo, no para competencia)
        // =====================================================================
        // Back + Y = Dynamic forward | Back + X = Dynamic reverse
        // Start + Y = Quasistatic forward | Start + X = Quasistatic reverse
        driverController.back().and(driverController.y()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(
            drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(
            drivetrain.sysIdQuasistatic(Direction.kReverse));

        // =====================================================================
        // Telemetria
        // =====================================================================
        drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // Test Controller (puerto 2) — prueba manual de mecanismos
        // =====================================================================
        // Usar SOLO para verificar mecanismos antes de usar el driver controller.
        // Cada mecanismo se controla de manera independiente con output manual.
        // Los flywheels son la excepcion: siempre giran juntos con signos opuestos.
        configureTestBindings();
    }

    /**
     * Configura los bindings del test controller (puerto 2).
     *
     * <p>Diseñado para sesiones de prueba de mecanismos. Cada mecanismo puede
     * ejercitarse de forma independiente sin usar el driver controller ni ejecutar
     * ningun automatismo.</p>
     *
     * <table border="1">
     *   <tr><th>Input</th><th>Mecanismo</th><th>Notas</th></tr>
     *   <tr><td>Right Trigger (eje)</td><td>Ambos flywheels</td><td>proporcional hasta FLYWHEEL_MANUAL_MAX_OUTPUT</td></tr>
     *   <tr><td>Left Trigger  (eje)</td><td>Intake Roller absorber</td><td>proporcional, negativo = CW = intake</td></tr>
     *   <tr><td>Left Bumper   (mantener)</td><td>Intake Roller reversa</td><td>expulsar / desatascar</td></tr>
     *   <tr><td>Left Stick Y  (|y|&gt;0.1)</td><td>Intake Pivot</td><td>manual arriba/abajo</td></tr>
     *   <tr><td>Right Stick Y (|y|&gt;0.1)</td><td>Hood</td><td>manual arriba/abajo</td></tr>
     *   <tr><td>A (mantener)</td><td>Indexer + Hopper adelante</td><td>corriente fija INDEXER_MANUAL_MAX_AMPS</td></tr>
     *   <tr><td>B (mantener)</td><td>Indexer + Hopper reversa</td><td>para desatascar</td></tr>
     * </table>
     */
    private void configureTestBindings() {

        // -----------------------------------------------------------------
        // Flywheels — ambos a la vez, signos opuestos
        // -----------------------------------------------------------------
        // Right Trigger actua como acelerador: cuanto mas se presiona, mas
        // rapido giran los flywheels. Al soltar, quedan en coast.
        testController.rightTrigger().whileTrue(
            new ManualFlywheelsCommand(shooter, testController::getRightTriggerAxis)
        );

        // -----------------------------------------------------------------
        // Intake Roller — Left Trigger (proporcional) + Left Bumper (reversa)
        // -----------------------------------------------------------------
        // Left Trigger: absorber pelotas (negativo = CW con CCW_Positive inversion = intake).
        // El eje (0..1) escala proporcional — mas presion = mas velocidad de succion.
        testController.leftTrigger().whileTrue(
            new ManualIntakeRollerCommand(intake,
                () -> -testController.getLeftTriggerAxis() * IntakeConstants.ROLLER_MANUAL_MAX_DUTY)
        );
        // Left Bumper: expulsar pelotas / desatascar (positivo = CCW = outtake).
        testController.leftBumper().whileTrue(
            new ManualIntakeRollerCommand(intake, () -> IntakeConstants.ROLLER_MANUAL_MAX_DUTY)
        );

        // -----------------------------------------------------------------
        // Indexer + Hopper — A (adelante) / B (reversa)
        // -----------------------------------------------------------------
        // A: alimentar pelota hacia los flywheels a corriente maxima manual.
        // B: reversa para desatascar o expulsar pelota del indexer.
        testController.a().whileTrue(
            new ManualIndexerCommand(shooter, () ->  ShooterConstants.INDEXER_MANUAL_MAX_AMPS)
        );
        testController.b().whileTrue(
            new ManualIndexerCommand(shooter, () -> -ShooterConstants.INDEXER_MANUAL_MAX_AMPS)
        );

        // -----------------------------------------------------------------
        // Intake Pivot — control de posicion manual por eje izquierdo
        // -----------------------------------------------------------------
        Trigger leftStickDeflected = new Trigger(
            () -> Math.abs(testController.getLeftY()) > 0.1
        );
        leftStickDeflected.whileTrue(
            new ManualIntakePivotCommand(intakePivot, () -> -testController.getLeftY())
        );

        // -----------------------------------------------------------------
        // Hood — control de angulo manual por eje derecho
        // -----------------------------------------------------------------
        Trigger rightStickDeflected = new Trigger(
            () -> Math.abs(testController.getRightY()) > 0.1
        );
        rightStickDeflected.whileTrue(
            new ManualHoodCommand(shooter, () -> -testController.getRightY())
        );
    }

    /**
     * Retorna el comando autonomo seleccionado en el SmartDashboard.
     * PathPlanner se encarga de seguir la trayectoria seleccionada.
     *
     * Si no hay ninguna auto seleccionada o AutoBuilder no esta configurado,
     * retorna null (Robot.java lo maneja de forma segura).
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

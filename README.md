# Horus — Robot FRC 6348 (Temporada 2026)

Codigo de competencia del robot **Horus** del equipo **6348**. Escrito en Java 17 con WPILib 2026, CTRE Phoenix 6 v26 y PhotonVision 2026.

---

## Arquitectura general

```
RobotContainer
├── CommandSwerveDrivetrain   (drivetrain Phoenix 6, swerve 4 ruedas)
├── VisionSubsystem           (4 camaras PhotonVision, 2 coprocesadores)
├── ShooterSubsystem          (flywheel + hood + indexer)
├── IntakeSubsystem           (roller TalonFX)
└── IntakePivotSubsystem      (pivot SparkMax/NEO)
```

---

## Tecnologias

| Libreria | Version | Uso |
|---|---|---|
| WPILib | 2026 | Base del robot, comandos, SmartDashboard |
| CTRE Phoenix 6 | v26.1.3 | TalonFX (Kraken X60), Pigeon 2, CANcoder |
| REVLib | 2026 | SparkMax + NEO (pivot del intake) |
| PathPlanner | 2026.1.2 | Trayectorias autonomas |
| PhotonVision | 2026.3.2 | Estimacion de pose por AprilTags |

---

## Estructura de paquetes

```
frc/robot/
├── Constants.java              ← Todas las constantes del robot
├── Robot.java                  ← Lifecycle del robot (TimedRobot)
├── RobotContainer.java         ← Subsistemas, bindings, PathPlanner
├── Telemetry.java              ← Telemetria del swerve (Mechanism2d)
│
├── Drive/
│   ├── CommandSwerveDrivetrain.java   ← Swerve + PathPlanner + SysId
│   ├── TunerConstants.java            ← Generado por Tuner X (hardware IDs)
│   └── commands/
│       └── AutoAimDriveCommand.java   ← Auto-aim con FieldCentricFacingAngle
│
├── Shooter/
│   ├── ShooterSubsystem.java          ← Flywheel, hood, indexer
│   ├── ShooterMath.java               ← Calculo de parametros de tiro
│   └── commands/
│       └── ShootCommand.java          ← Flujo completo de disparo + retraccion progresiva del pivot
│
├── Intake/
│   ├── IntakeSubsystem.java           ← Roller (TalonFX)
│   ├── IntakePivotSubsystem.java      ← Pivot (SparkMax/NEO)
│   └── commands/
│       ├── RunIntakeCommand.java
│       ├── DeployPivotCommand.java
│       └── RetractPivotCommand.java
│
└── Vision/
    ├── VisionSubsystem.java           ← Fusion de vision con odometria
    └── CameraRunner.java              ← Hilo daemon por camara
```

---

## Controles (Driver Controller — puerto 0)

| Boton | Accion |
|---|---|
| Joystick izquierdo | Traslacion field-centric (X/Y) |
| Joystick derecho X | Rotacion (omega) |
| Right Trigger | Disparar (vision-based + retraccion progresiva del pivot) |
| Left Trigger | Correr roller del intake |
| X | Auto-aim (robot apunta al objetivo, driver controla traslacion) |
| Y | Resetear heading field-centric |
| Right Bumper | Desplegar intake |
| Left Bumper | Retraer intake |
| Back + Y/X | SysId Dynamic forward/reverse |
| Start + Y/X | SysId Quasistatic forward/reverse |

---

## Subsistemas

### Drive — Swerve 4 ruedas (Kraken X60)

- **Odometria a 250 Hz** en CANivore para maxima precision de posicion.
- Fusion de vision con filtro de Kalman (`addVisionMeasurement`).
- PathPlanner configurado con feedforwards de rueda para mejor seguimiento.
- Auto-aim: `FieldCentricFacingAngle` + heading controller PID.

### Vision — 4 camaras PhotonVision

- 2 coprocesadores (`10.63.48.11`, `10.63.48.12`), 2 camaras cada uno.
- Cada camara corre en hilo daemon independiente (`CameraRunner`).
- `MULTI_TAG_PNP_ON_COPROCESSOR` con fallback `LOWEST_AMBIGUITY`.
- Confianza dinamica por camara: aumenta con frames consistentes, decae sin targets.
- Desviaciones estandar escaladas por distancia, numero de tags y nivel de confianza.

### Shooter — Flywheel + Hood + Indexer

- **Flywheel** (x2 Kraken X60): `VelocityVoltage + FOC`, kS+kV+kA para feedforward optimo.
- **Hood** (Kraken X60): `MotionMagicVoltage + FOC`, kG `Arm_Cosine` para compensacion de gravedad.
- **Indexer** (Kraken X60): `DutyCycleOut` simple.
- Signals cacheadas + `BaseStatusSignal.refreshAll()` en periodic para reducir trafico CAN.
- `optimizeBusUtilization()` en los 4 motores.

### Flujo de disparo (ShootCommand)

```
Presionar Right Trigger
  ↓
¿Hay AprilTags visibles?
  ├── SI → Calcular RPM + angulo con ShooterMath (actualiza cada 20 ms)
  └── NO → Modo default: RPM fija + hood en home
  ↓
¿Flywheel a velocidad Y hood en angulo? → Activar indexer
  ↓
Indexer encendido → Iniciar retraccion progresiva del pivot (ratchet)
  ↓
Soltar boton → Detener indexer, coast flywheel, home hood, restaurar pivot
```

### Retraccion progresiva del pivot durante el disparo

Cuando el indexer enciende (primera pelota en camino), el pivot inicia una retraccion gradual en pasos:

1. **Fase UP**: el pivot sube `PIVOT_RATCHET_STEP_UP_ROTATIONS` hacia retractado y espera `PIVOT_RATCHET_UP_SECONDS`.
2. **Fase DOWN**: el pivot baja `PIVOT_RATCHET_STEP_DOWN_ROTATIONS` (menor que UP) y espera `PIVOT_RATCHET_DOWN_SECONDS`.
3. Se repite hasta llegar a `PIVOT_RETRACTED_POSITION_ROTATIONS`. El efecto neto por ciclo es `STEP_UP - STEP_DOWN` rotaciones hacia retractado.
4. Al soltar el boton, el pivot regresa a la posicion que tenia **antes** de empezar a disparar.

### Compensacion de gravedad del pivot

El SparkMax PID aplica un feedforward constante de `PIVOT_GRAVITY_FF_VOLTS` voltios (via `ArbFFUnits.kVoltage`) en todo momento en que el PID esta activo. Esto compensa el peso del brazo del intake que lo jalaria hacia la posicion desplegada.

---

## Seleccion de campo (SmartDashboard)

El codigo soporta dos variantes de campo: **Welded** (default) y **Andymark**.

Para cambiar:
1. Abrir SmartDashboard o Shuffleboard.
2. Ir a la tabla **Preferences**.
3. Cambiar `IsAndymarkField` a `true` (Andymark) o `false` (Welded).
4. **Reiniciar el codigo del robot** para que el cambio tome efecto.

El tipo de campo activo se muestra en `Robot/FieldType`.

---

## SmartDashboard — entradas publicadas

### Robot (global)
| Clave | Descripcion |
|---|---|
| `Robot/FieldType` | Tipo de campo activo (`WELDED` o `ANDYMARK`) |
| `Robot/Alliance` | Alianza actual (`Red`, `Blue`, `None`) |
| `Robot/RobotMode` | Modo actual (`Disabled`, `Autonomous`, `Teleop`, `Test`) |

### Shooter
| Clave | Descripcion |
|---|---|
| `Shooter/LeftRPM` | Velocidad flywheel izquierdo [RPM] |
| `Shooter/RightRPM` | Velocidad flywheel derecho [RPM] |
| `Shooter/TargetRPM` | RPM objetivo |
| `Shooter/RPMError` | Error de velocidad [RPM] |
| `Shooter/HoodAngleDeg` | Angulo actual del hood [grados] |
| `Shooter/TargetAngleDeg` | Angulo objetivo del hood [grados] |
| `Shooter/LeftStatorAmps` | Corriente flywheel izquierdo [A] |
| `Shooter/RightStatorAmps` | Corriente flywheel derecho [A] |
| `Shooter/HoodStatorAmps` | Corriente hood [A] |
| `Shooter/IndexerStatorAmps` | Corriente indexer [A] |
| `Shooter/FlywheelAtSpeed` | `true` si flywheel en RPM objetivo |
| `Shooter/HoodAtAngle` | `true` si hood en angulo objetivo |
| `Shooter/HoodAtHome` | `true` si hood en posicion home |
| `Shooter/SpinningUp` | `true` si flywheel acelerando (aun no a velocidad) |
| `Shooter/IndexerRunning` | `true` si indexer activo |
| `Shooter/ReadyToShoot` | `true` si flywheel + hood listos |

### Intake
| Clave | Descripcion |
|---|---|
| `Intake/MotorOutput` | Salida actual del roller [%] |
| `Intake/StatorAmps` | Corriente de stator [A] |
| `Intake/IsRunning` | `true` si roller activo |

### IntakePivot
| Clave | Descripcion |
|---|---|
| `IntakePivot/Position` | Posicion actual [rot mecanismo] |
| `IntakePivot/Setpoint` | Setpoint objetivo [rot mecanismo] |
| `IntakePivot/AtSetpoint` | `true` si en tolerancia del setpoint |

### AutoAim
| Clave | Descripcion |
|---|---|
| `AutoAim/TargetHeading` | Heading objetivo calculado [grados] |
| `AutoAim/CurrentHeading` | Heading actual del robot [grados] |
| `AutoAim/Aligned` | `true` si el robot esta alineado con el objetivo |
| `AutoAim/Zone` | Zona del campo (`RED_ALLIANCE`, `NEUTRAL`, `BLUE_ALLIANCE`) |
| `AutoAim/ValidTarget` | `true` si hay un objetivo de disparo valido |

### Vision (por camara)
| Clave | Descripcion |
|---|---|
| `Vision/TagsVisible` | Numero total de tags visibles |
| `Vision/CAM_NAME/HasTargets` | `true` si la camara ve algun tag |
| `Vision/CAM_NAME/TagCount` | Numero de tags que ve esa camara |
| `Vision/CAM_NAME/Confidence` | Nivel de confianza dinamica [0.0–1.0] |
| `Vision/CAM_NAME/EstX` | Posicion X estimada [m] |
| `Vision/CAM_NAME/EstY` | Posicion Y estimada [m] |

---

## Convenciones del codigo

| Aspecto | Convencion |
|---|---|
| Constantes | `UPPER_SNAKE_CASE` — sin prefijos `k` ni `m` |
| Variables/metodos | `camelCase` — nombres completamente descriptivos |
| Clases | `PascalCase` |
| Documentacion Javadoc | En espanol |
| Codigo fuente | En ingles (identificadores, strings tecnicos) |
| Paquetes | `frc.robot.Subsistema` |

---

## Como compilar

Requiere WPILib 2026 instalado en `C:/Users/Public/wpilib/2026/`.

```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew compileJava
```

Para desplegar al robot:
```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew deploy
```

---

## Calibraciones pendientes (TODO)

Buscar `// TODO` en el codigo para encontrar todos los valores que requieren medicion o tuning en el robot fisico. Los principales son:

| Subsistema | Pendiente |
|---|---|
| Shooter | Medir `SHOOTER_HEIGHT_METERS`, `FLYWHEEL_RADIUS_METERS`, `SHOOTER_X_OFFSET_METERS` |
| Shooter | Tunear `FLYWHEEL_P/V/S`, `HOOD_P/G`, `HOOD_GEAR_RATIO` |
| Hood | Medir angulos `MIN_SHOT_ANGLE_DEGREES`, `MAX_SHOT_ANGLE_DEGREES`, `HOOD_HOME_ANGLE_DEGREES` |
| Intake Pivot | Calibrar `PIVOT_DEPLOYED_POSITION_ROTATIONS`, `PIVOT_GEAR_RATIO` |
| Intake Pivot | Calibrar `PIVOT_RATCHET_STEP_UP_ROTATIONS`, `PIVOT_RATCHET_STEP_DOWN_ROTATIONS`, `PIVOT_RATCHET_UP_SECONDS`, `PIVOT_RATCHET_DOWN_SECONDS` |
| Intake Pivot | Tunear `PIVOT_GRAVITY_FF_VOLTS` para compensar el peso del brazo |
| Vision | Medir `ROBOT_TO_CAM_*` con cinta desde el centro del robot |
| PathPlanner | Tunear `PATHPLANNER_TRANSLATION_P`, `PATHPLANNER_ROTATION_P` |
| Auto-aim | Tunear `AUTO_AIM_P` para velocidad de giro |

---

*Equipo 6348 Horus — FRC Temporada 2026*

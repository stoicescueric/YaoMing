# YaoMing

Robot code for **Quantum Robotics #14270** (Bucharest, Romania) in the 2025–2026 FIRST Tech Challenge season, DECODE.

![YaoMing, the Quantum Robotics DECODE robot](doc/media/yaoming.png)

## Abstract

YaoMing is a turreted flywheel shooter on a mecanum drivetrain. It collects the game's purple and green artifacts, tracks the goal with its turret while driving, and fires three-ball bursts without stopping. This repository holds everything that runs on the robot: localization, path following, the shoot-on-the-move aiming solver, flywheel control, the intake and launcher state machines, vision, and the autonomous routines.

With this code, in the DECODE season the team:

- won the **Romanian National Championship** as Winning Alliance Captain, and took the Design Award;
- won the **Ross Division at the FIRST Championship** in Houston as Winning Alliance Captain, after ranking 1st of 56 teams in qualification;
- finished **3rd in the Championship finals**, played between the six division-winning alliances;
- received the **Think Award** at the FIRST Championship;
- set world-record scores during the season and ended it ranked **1st worldwide in OPR** across official events.

The team is run by high-school students, from robot design to software to fundraising.

## Results

| Event | Date | Result |
|---|---|---|
| Romania South League Tournament | Jan 31 – Feb 1, 2026 | Ranked 1st of 26, Inspire Award 2nd place |
| Romania Championship | Mar 13–15, 2026 | Winning Alliance Captain, Design Award |
| FIRST Championship, Ross Division | Apr 29 – May 2, 2026 | Ranked 1st of 56, Winning Alliance Captain, Think Award |
| FIRST Championship, finals | May 2, 2026 | 3rd place |
| Multinational Tech Invitational (off-season) | Summer 2026 | Finalist Alliance |

Official match data: [FIRST event results for team 14270](https://ftc-events.firstinspires.org/2025/team/14270).

![YaoMing shooting on the Ross Division field at the FIRST Championship](doc/media/yaoming-ross-field.png)

*YaoMing mid-burst on the Ross Division field in Houston.*

## In the press

**Forbes România, 30 sub 30 (2026 edition).** Forbes included the team in its 30 Under 30 list and profiled the season:
[Echipa Quantum Robotics: Podiumul mondial, construit de liceeni](https://www.forbes.ro/forbes-romania-30-sub-30-editia-2026-echipa-quantum-robotics-podiumul-mondial-construit-de-liceeni-529782) (Irina Pană, November 4, 2026).

## What is different about this robot

### It shoots while moving

Most robots stop, aim, and shoot. YaoMing aims at a *virtual* target that is shifted to cancel the robot's own motion, so the ball lands in the real goal even though it leaves a moving robot.

The solver in `Hardware/Sensors.java` splits the robot's velocity into two parts relative to the goal. Motion toward or away from the goal changes the effective distance, which sets flywheel speed and hood angle. Motion across the goal changes the angle, which sets the turret lead. Flight time depends on distance and distance depends on the correction, so the solver repeats the calculation a few times until the flight time stops changing. Feeder delay and turret lag are added as fixed latencies.

### Flywheel speed and hood angle come from measured shots

`Hardware/Outtake/Launcher.java` keeps tables of distance against flywheel velocity and hood position, taken from real shots on the field. A monotone cubic spline (`Util/Wrapper/InterpLUT.java`) interpolates between the points, so the curve never overshoots between two measurements.

### The flywheel is controlled from an identified model

Each ball slows the wheel down, and the next ball in the burst needs it back at speed. The repo contains three controllers that can be swapped at runtime:

- a PIDF velocity controller with battery-voltage compensation;
- a two-mode controller (full power to spin up, then feedforward with a small proportional hold) modelled on FRC Team 254's approach;
- an RST pole-placement controller (`Util/Controllers/RSTFlywheelController.java`), designed from open-loop step tests of the real wheel. The recorded runs are in `flywheel-sysid/`. Its poles are placed real, so the wheel does not overshoot, and its gains are recomputed from the measured loop time each cycle. This one is post-season work.

Battery voltage is low-pass filtered and frozen at the start of a burst, so the sag caused by the shot itself does not disturb the controller.

### The flywheel spins up before the robot reaches its shooting spot

While the robot is still driving toward the launch zone, the launcher predicts where it will enter the zone and spins the wheel to the speed needed from that point. The first ball can leave as soon as the robot crosses the line.

### A turret that tracks the goal continuously

Two servos drive the turret (`Hardware/Outtake/Turret.java`). It computes the field angle to the virtual target every loop, subtracts the robot's heading, and can lead by angular velocity to cover servo lag during fast turns.

### Vision that picks where to collect

A Limelight 3A runs a neural detector trained on the two artifact colours, plus a Python pipeline (`LimelightVisionProcessing.py`) that tracks each ball between frames, estimates its velocity, and splits the image into zones. The robot steers toward the zone holding the most artifacts, with hysteresis so it does not flicker between two similar zones.

### A fast, measured control loop

The Control Hub is read once per loop with manual bulk caching, and every motor and servo is wrapped so that a command is sent only when its value changes (`Util/Caching/`). The loop times each subsystem separately and reports it to the dashboard, which is how slow code gets found.

### Intake that knows how many balls it holds

Laser distance sensors act as presence detectors along the transfer, with debounce and glitch filtering. A current threshold on the intake motor detects stalls and triggers an unjam. LEDs show the driver the fill state.

## Tech stack

| Layer | What we use |
|---|---|
| Language and platform | Java on Android, REV Control Hub, FTC SDK 11.1 |
| Path following | [blob](https://github.com/PzmuV1517/blob), a path follower that started inside this project; [Pedro Pathing](https://pedropathing.com) 2.0.4 poses and constants |
| Localization | goBILDA Pinpoint dead-wheel odometry (OctoQuad selectable from one config enum) |
| Vision | Limelight 3A, TensorFlow Lite detector, Python/OpenCV pipeline |
| Control | Custom PID, PIDF, PDFL and RST controllers; Kalman and low-pass filters; slew-rate limiter; regression utilities |
| Tuning and telemetry | FTC Dashboard 0.5.1, Panels, live-editable `@Config` constants |
| Utilities | FTCLib core 2.1.1 |
| Build | Gradle, Android Studio, hot-reload deploys of team code (timings in `pusher-reports/`) |

## Repository layout

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Hardware/
│   ├── Robot.java            main loop, bulk caching, per-subsystem profiling
│   ├── Sensors.java          pose, target geometry, shoot-on-the-move solver
│   ├── BlobDrive.java        drivetrain and path follower adapter
│   ├── Intake/               intake and transfer state machine
│   ├── Outtake/              launcher, turret, hood
│   └── Vision/               Limelight interface and zone selection
├── OpMode/
│   ├── Auto/                 autonomous routines (far side and close side)
│   ├── TeleOp/               driver-controlled modes, red and blue
│   └── Tuning/               system identification and tuning OpModes
└── Util/                     controllers, filters, caching wrappers, math
LimelightVisionProcessing.py  Limelight Python pipeline
flywheel-sysid/               recorded flywheel step-response data
configs/                      robot hardware configurations
```

## Build and deploy

1. Install Android Studio Ladybug (2024.2) or newer.
2. Clone the repository and open the project root.
3. Connect to the Control Hub over USB or Wi-Fi and run the `TeamCode` configuration.

The code expects YaoMing's hardware configuration. The files in `configs/` list every device name.

## Paper

The launch model and the shoot-on-the-move compensation are described in a preprint:
*Robust Drag-Aware Launch Planning and Shoot-on-the-Move Compensation for a Mobile Robot*, [doi.org/10.20944/preprints202610.2907.v1](https://doi.org/10.20944/preprints202610.2907.v1).

## Team and contact

Quantum Robotics #14270 is based at the International Computer High School of Bucharest and has competed in FIRST Tech Challenge since 2017.

- Website: [qrobotics.eu](https://qrobotics.eu)
- Code maintainer: [stoicescueric](https://github.com/stoicescueric)

Press and sponsorship enquiries are welcome through the website.

## License

Built on the [FIRST Tech Challenge SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) and distributed under the BSD 3-Clause Clear License. See [LICENSE](LICENSE).

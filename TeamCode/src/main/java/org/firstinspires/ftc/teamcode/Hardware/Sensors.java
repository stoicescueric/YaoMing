package org.firstinspires.ftc.teamcode.Hardware;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Util.Math.Debouncer;
import org.firstinspires.ftc.teamcode.Util.Wrapper.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.Util.Wrapper.InterpLUT;
import org.firstinspires.ftc.teamcode.blob.math.LowPassFilter;

@Config
public class Sensors {
    private Robot robot;

    CachingServo light;
    InterpLUT shotTime;

    public enum LightColor {
        OFF(0),
        RED(0.3),
        GREEN(0.5),
        BLUE(0.611);
        final double value;
        LightColor(double value) {
            this.value = value;
        }
    }
    public LightColor lightColor = LightColor.OFF;
    Pose pose;
    double currentX,currentY,currentHeading;
    public static double FORWARD_TURRET_OFFSET = -1.66;

    private double currentVelocityShooter = 0;
    private double voltage;

    public long readVoltageTime = 0;
    private long lastUpdateTimeNs = 0;

    public static double alphaVoltageFilter = 0.08;
    private LowPassFilter voltageFilter = new LowPassFilter(alphaVoltageFilter, 0);

    private boolean intakeMotor1OverCurrent = false;
    private boolean breakBeamPos1High = false;
    private boolean breakBeamPos2High = false;
    private boolean breakBeamPos3High = false;

    public  double targetX = -66.6;
    public  double targetY = -65;
    public static double targetXRedClose = -71;
    public static double targetYRedClose = 67;
    public double targetXBlueClose = -71;
    public double targetYBlueClose = -67;

    public static double targetXRedFar = -72;
    public static double servoPos = 0.4;
    public static double targetYRedFar = 64;
    public double targetXBlueFar = -72;
    public double targetYBlueFar = -64;
    public double virtualTargetX = targetX;
    public double virtualTargetY = targetY;

    public double intakeSpeed;
    double shooterWorldX, shooterWorldY;

    public static double STILL_MAX_TRANSLATIONAL_SPEED = 13; // field units per second
    public static double STILL_MAX_ANGULAR_SPEED = 12; //radians per seconds
    private double lastStillX = Double.NaN;
    private double lastStillY = Double.NaN;
    private double lastStillHeading = Double.NaN;
    private long lastStillCheckLoopTimeNs = 0; // Robot.loopTime at last history update

    public static double CLOSEZONE_X1 = -89, CLOSEZONE_Y1 = 89;
    public static double CLOSEZONE_X2 = 15,  CLOSEZONE_Y2 = 0;
    public static double CLOSEZONE_X3 = -89, CLOSEZONE_Y3 = -89;
    public static double FARZONE_X1 = 63, FARZONE_Y1 = 24;
    public static double FARZONE_X2 = 44,  FARZONE_Y2 = 0;
    public static double FARZONE_X3 = 63,  FARZONE_Y3 = -24;

    public double projectedX,projectedY;

    public boolean sotm = false;

    public static double switchTarget = 25;
    public static double SLOWZONE_X1 = 10, SLOWZONE_Y1 = 35;
    public static double SLOWZONE_X2 = 35, SLOWZONE_Y2 = 35;
    public static double SLOWZONE_X3 = 10, SLOWZONE_Y3 = 72;
    public static double SLOWZONE_X4 = 10, SLOWZONE_Y4 = 72;

    private double velX = 0.0;
    private double velY = 0.0;
    public double xVelocityRobot, yVelocityRobot;
    public double xAccRobot, yAccRobot;
    public static double filterParameter = 0.65;
    private  final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double DEFAULT_PROJECTILE_SPEED = 300.0; // inches per second, rough guess

    private DigitalChannel breakBeamPos1, breakBeamPos2, breakBeamPos3;
    private Debouncer bb1,bb2,bb3;
    public boolean lastValueBreakBreamPos1,lastValuebreamBeamPos2,lastValuebreamBeamPos3;

    public long firstTrueBeam1,firstTrueBeam2,firstTrueBeam3;

    // --- SOTM / TURRET PREDICTION VARIABLES ---
    public static double ACCEL_COMP_FACTOR = 0;     // Multiplier for the 0.5*a*t^2 term
    public static double SOTM_GAIN = 0.9;            // Multiplier for physical momentum loss
    public static double SHOOTER_FEEDER_DELAY = 0.05; // Sec: Delay from 'fire' to ball exit
    public static double TURRET_MECH_LOOKAHEAD_S = 0.1; // Sec: Delay for Turret slew lag
    public static double VELO_THRESHOLD = 4.0;        // Rad/s: Deadband for lookahead
    public static double PHYSICS_SHOT_TIME_EPS = 0.05;// Convergence threshold for the solver

    private double lastTurretAngle = 0;
    private double turretAngleVelo = 0;
    private ElapsedTime turretAngleTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public Sensors(Robot robot) {
        this.robot = robot;
        initSensors();
        if(Info.alliance == Alliance.BLUE){
            targetX = targetXBlueClose;
            targetY = targetYBlueClose;
        }else {
            targetX = targetXRedClose;
            targetY = targetYRedClose;
        }
        shotTime = new InterpLUT();
        createShotTime();
    }

    void createShotTime() {
        shotTime.add(50,0.5);
        shotTime.add(60,0.54);
        shotTime.add(76,0.6);
        shotTime.add(85,0.7);
        shotTime.add(95,0.8);
        shotTime.createLUT();
    }

    public static boolean usePredictivePose = true;
    public static double timeLatencyTurret = 0.315; //sec
    public static double debouncerTime = 50;

    private void initSensors() {
        turretAngleTimer.reset(); // Safely reset the timer initialized at the class level
        bb1 = new Debouncer(debouncerTime, Debouncer.DebounceType.kBoth);
        bb2 = new Debouncer(debouncerTime, Debouncer.DebounceType.kBoth);
        bb3 = new Debouncer(debouncerTime, Debouncer.DebounceType.kBoth);
        light = new CachingServo(robot.hw.get(Servo.class,"led"));
        voltageSensor = new CachingVoltageSensor(robot.hw);

        breakBeamPos1 = robot.hw.get(DigitalChannel.class, "beamBrakePos1");
        breakBeamPos1.setMode(DigitalChannel.Mode.INPUT);
        lastValueBreakBreamPos1 = breakBeamPos1.getState();

        breakBeamPos2 = robot.hw.get(DigitalChannel.class, "beamBrakePos2");
        breakBeamPos2.setMode(DigitalChannel.Mode.INPUT);
        lastValuebreamBeamPos2= breakBeamPos2.getState();

        breakBeamPos3 = robot.hw.get(DigitalChannel.class, "beamBrakePos3");
        breakBeamPos3.setMode(DigitalChannel.Mode.INPUT);
        lastValuebreamBeamPos3= breakBeamPos3.getState();

        voltage = robot.hw.voltageSensor.iterator().next().getVoltage();
        readVoltageTime = System.currentTimeMillis();
        lastUpdateTimeNs = System.nanoTime();
    }

    public double getTargetX(){
        if(sotm) return virtualTargetX;
        return targetX;
    }

    public double getTargetY(){
        if(sotm) return virtualTargetY;
        return targetY;
    }

    CachingVoltageSensor voltageSensor;

    public double getDistanceBetweenPoints(double x1,double x2,double y1,double y2){
        return Math.sqrt((x1 - x2) * (x1-x2) + (y1 - y2) * (y1 - y2));
    }

    public static double threesholdTime = 0.050;
    public static double latencyFactor = 0.07;
    public static double projectedXSign = 1;
    public static boolean useFixedTof = false;
    public static double fixedTOF = 0.6;
    public static double projectedYSign = 1;
    public static double convergenceLoopCnt = 5;

    double interpolatedTOF;
    double virtualDistance;

    public boolean isFarZone() {
        return currentX > switchTarget;
    }

    public void update() {
        Pose prevPose = pose;

        double prevX = currentX;
        double prevY = currentY;

        long prevUpdateTimeNs = lastUpdateTimeNs;
        pose = robot.blob.odo.getPose();

        currentX = pose.getX();
        currentY = pose.getY();
        currentHeading = pose.getHeading();

        calculateVelAndAcc();

        if(isFarZone()) {
            if(Info.alliance == Alliance.RED) {
                targetX = targetXRedFar;
                targetY = targetYRedFar;
            } else {
                targetX = targetXBlueFar;
                targetY = targetYBlueFar;
            }
        } else {
            if(Info.alliance == Alliance.RED) {
                targetX = targetXRedClose;
                targetY = targetYRedClose;
            } else {
                targetX = targetXBlueClose;
                targetY = targetYBlueClose;
            }
        }

        shooterWorldX = currentX + (FORWARD_TURRET_OFFSET * Math.cos(currentHeading));
        shooterWorldY = currentY + (FORWARD_TURRET_OFFSET * Math.sin(currentHeading));

        if(sotm) {
            double deltaX = targetX - getX();
            double deltaY = targetY - getY();
            virtualDistance = Math.hypot(deltaX,deltaY);
            interpolatedTOF = shotTime.get(virtualDistance);
            if(useFixedTof) interpolatedTOF = fixedTOF;

            for(int i = 0;i<convergenceLoopCnt;i++) {
                double t = SHOOTER_FEEDER_DELAY + interpolatedTOF;
                double t2 = t * t;

                double displaceX = ((xVelocityRobot * t) + (0.5 * xAccRobot * t2 * ACCEL_COMP_FACTOR)) * SOTM_GAIN;
                double displaceY = ((yVelocityRobot * t) + (0.5 * yAccRobot * t2 * ACCEL_COMP_FACTOR)) * SOTM_GAIN;

                virtualTargetX = targetX + (displaceX * sotmDirX);
                virtualTargetY = targetY + (displaceY * sotmDirY);
                double newVirtualDistance = Math.hypot(virtualTargetX - getX(), virtualTargetY - getY());
                double newTOF = shotTime.get(newVirtualDistance);
                if(useFixedTof) newTOF = fixedTOF;


                if (Math.abs(newTOF - interpolatedTOF) < PHYSICS_SHOT_TIME_EPS) break;

                virtualDistance = newVirtualDistance;
                interpolatedTOF = newTOF;
            }
        }

        calculateDistance();

        // Breakbeam updates
        if (breakBeamPos1 != null) {
            lastValueBreakBreamPos1 = breakBeamPos1High;
            breakBeamPos1High = bb1.calculate(!(breakBeamPos1.getState()));
            if(breakBeamPos1High && !lastValueBreakBreamPos1) {
                firstTrueBeam1 = System.currentTimeMillis();
            }else if(!breakBeamPos1High && !lastValueBreakBreamPos1) {
                firstTrueBeam1 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos1High = false;
        }

        if (breakBeamPos2 != null) {
            lastValuebreamBeamPos2 = breakBeamPos2High;
            breakBeamPos2High = bb2.calculate(!(breakBeamPos2.getState()));
            if(breakBeamPos2High && !lastValuebreamBeamPos2) {
                firstTrueBeam2 = System.currentTimeMillis();
            }else if(!breakBeamPos2High && !lastValuebreamBeamPos2) {
                firstTrueBeam2 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos2High = false;
        }

        if (breakBeamPos3 != null) {
            lastValuebreamBeamPos3 = breakBeamPos3High;
            breakBeamPos3High = bb3.calculate(!(breakBeamPos3.getState()));

            if(breakBeamPos3High && !lastValuebreamBeamPos3) {
                firstTrueBeam3 = System.currentTimeMillis();
            }else if(!breakBeamPos3High && !lastValuebreamBeamPos3) {
                firstTrueBeam3 = System.currentTimeMillis();
            }
        } else {
            breakBeamPos3High = false;
        }

        if(!sotm) shooterAngle = Math.atan2(getTargetY() - (shooterWorldY + velY * timeLatencyTurret), getTargetX() - (shooterWorldX + velX * timeLatencyTurret));
        else shooterAngle = Math.atan2(getTargetY() - shooterWorldY, getTargetX() - shooterWorldX);
        if(lookUpTurret) shooterAngle = updateTurretPrediction(shooterAngle, 0);

        light.setPosition(lightColor.value);
        voltage = voltageFilter.getValue(voltageSensor.getVoltage());
    }

    public static boolean lookUpTurret = false;
    public static double rpmTimeLatency = 0.1;

    /**
     * Calculates the predicted turret angle based on current velocity
     * to compensate for mechanical latency.
     */
    public double updateTurretPrediction(double baseTurretAngle, double compensationAngle) {
        double turretAngle = baseTurretAngle + compensationAngle;
        double taDt = turretAngleTimer.seconds();

        // 1. Calculate Velocity (Change in angle over change in time)
        if (taDt > 0.001) {
            double deltaAngle = turretAngle - lastTurretAngle;

            while (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
            while (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

            turretAngleVelo = deltaAngle / taDt;
            lastTurretAngle = turretAngle;
            turretAngleTimer.reset();
        }

        // 2. Apply Lookahead/Lead
        if (Math.abs(turretAngleVelo) > VELO_THRESHOLD) {
            turretAngle += (turretAngleVelo * TURRET_MECH_LOOKAHEAD_S);
        }

        return turretAngle;
    }


    void calculateDistance() {
        double dx = getTargetX() - (shooterWorldX + velX * rpmTimeLatency);
        double dy = getTargetY() - (shooterWorldY + velY * rpmTimeLatency);
        shooterDistanceBackboard =  Math.sqrt(dx * dx + dy * dy);

        dx = targetX - currentX;
        dy = targetY -currentY;
        distanceToBackBoard =  Math.sqrt(dx * dx + dy * dy);
    }

    public double getVoltage() { return voltage; }
    public Pose getPose() { return pose; }
    public double getX() { return currentX; }

    public void calculateVelAndAcc() {
        xAccRobot = robot.blob.odo.getAccVector().getX();
        yAccRobot = robot.blob.odo.getAccVector().getY();

        velX = robot.blob.getVelocityX();
        velY  = robot.blob.getVelocityY();

        xVelocityRobot = velX;
        yVelocityRobot = velY;
    }

    public void toggleSOTM(boolean val) { sotm = val; }
    public double getMoveGoalX() { return virtualTargetX; }
    public double getMoveGoalY() { return virtualTargetY; }

    double distanceToBackBoard;
    double shooterDistanceBackboard;
    public static double sotmDirY = -1;
    public static double sotmDirX = -1;
    public double getDistanceToBackboard() { return distanceToBackBoard; }
    public double getShooterDistanceToBackboard() { return shooterDistanceBackboard; }

    double shooterAngle;
    public double getShooterAngleToTarget() { return shooterAngle; }
    public double getAngularVelocity() { return robot.blob.odo.getAngularVelocity(); }
    public double getY() { return currentY; }
    public double getVelocityIntake() { return intakeSpeed; }
    public double getHeading() { return currentHeading; }
    public double getVelocity() { return currentVelocityShooter; }

    public boolean isIntakeMotor1OverCurrent() { return intakeMotor1OverCurrent; }

    public boolean shootingLong() {
        return currentX > OuttakePositions.FAR_ZONE_X_THRESHOLD;
    }

    public boolean isInTargetZone(double x, double y) {
        if (isPointInTriangle(x, y, CLOSEZONE_X1, CLOSEZONE_Y1, CLOSEZONE_X2, CLOSEZONE_Y2, CLOSEZONE_X3, CLOSEZONE_Y3)) {
            return true;
        }
        if (isPointInTriangle(x, y, FARZONE_X1, FARZONE_Y1, FARZONE_X2, FARZONE_Y2, FARZONE_X3, FARZONE_Y3)) {
            return true;
        }
        return false;
    }

    private boolean isPointInTriangle(double x, double y, double x1, double y1, double x2, double y2, double x3, double y3) {
        double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
        if (denom == 0) return false;

        double a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
        double b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
        double c = 1 - a - b;

        double eps = 1e-6;
        return a >= -eps && b >= -eps && c >= -eps;
    }

    public void updateTargetForZone() {
        boolean isFar = currentX > OuttakePositions.FAR_ZONE_X_THRESHOLD;
        if (Info.alliance == Alliance.BLUE) {
            targetX = isFar ? targetXBlueFar : targetXBlueClose;
            targetY = isFar ? targetYBlueFar : targetYBlueClose;
        } else {
            targetX = isFar ? targetXRedFar : targetXRedClose;
            targetY = isFar ? targetYRedFar : targetYRedClose;
        }
    }

    public void setLedColor(LightColor state) { lightColor = state; }
    public double getVelX() { return xVelocityRobot; }
    public double getVelY() { return yVelocityRobot; }

    public double estimateShotTimeOfFlight(double distance) {
        double speed = DEFAULT_PROJECTILE_SPEED;
        if (speed <= 1e-3) return 0.0;
        return Math.max(0.0, distance / speed);
    }

    public boolean isInSlowZone() {
        double[] xs = { SLOWZONE_X1, SLOWZONE_X2, SLOWZONE_X3, SLOWZONE_X4 };
        double[] ys;

        if (Info.alliance == Alliance.BLUE) {
            ys = new double[]{ -SLOWZONE_Y1, -SLOWZONE_Y2, -SLOWZONE_Y3, -SLOWZONE_Y4 };
        } else {
            ys = new double[]{ SLOWZONE_Y1, SLOWZONE_Y2, SLOWZONE_Y3, SLOWZONE_Y4 };
        }

        return isPointInPolygon(currentX, currentY, xs, ys);
    }

    private boolean isPointInPolygon(double x, double y, double[] xs, double[] ys) {
        boolean inside = false;
        int n = xs.length;
        if (n < 3) return false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = xs[i], yi = ys[i];
            double xj = xs[j], yj = ys[j];

            boolean intersect = ((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    public boolean isBreakBeamPos1Low(){ return breakBeamPos1High; }
    public boolean isBreakBeamPos2Low(){ return breakBeamPos2High; }
    public boolean isBreakBeamPos3Low(){ return breakBeamPos3High; }

    public double getHowLongBeam3() { return System.currentTimeMillis() - firstTrueBeam3; }
    public double getHowLongBeam2() { return System.currentTimeMillis() - firstTrueBeam2; }
    public double getHowLongBeam1() { return System.currentTimeMillis() - firstTrueBeam1; }

    public double getDistanceFromPose(Pose pose) {
        double dx = getTargetX() - pose.getX();
        double dy = getTargetY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getShooterX() { return shooterWorldX; }
    public double getShooterY() { return shooterWorldY; }
    public void setUsePredictivePose(boolean use) { usePredictivePose = use; }

    public boolean areAllBeamsLowForTime() {
        return getHowLongBeam1() > IntakeConstants.beam1stopDelay &&
                getHowLongBeam2() > IntakeConstants.beam2stopDelay &&
                getHowLongBeam3() > IntakeConstants.beam3StopDelay;
    }
}
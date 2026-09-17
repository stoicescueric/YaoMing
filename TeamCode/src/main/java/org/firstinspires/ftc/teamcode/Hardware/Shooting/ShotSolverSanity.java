package org.firstinspires.ftc.teamcode.Hardware.Shooting;

/**
 * Sanity checks for {@link ShotSolver}, runnable on a plain JVM.
 *
 * <p>Interception maths fails quietly: a sign error still produces a plausible-looking angle, and no
 * amount of field testing isolates it cleanly. These assertions are made against relationships that
 * must hold whatever the constants are, so they stay valid when the ball and robot change.
 *
 * <p>It has a {@code main} and never runs on the robot. To run it, from the repo root:
 * <pre>
 *   SRC=TeamCode/src/main/java
 *   CP=&lt;dashboard classes.jar&gt;:&lt;blob classes.jar&gt;
 *   javac -cp "$CP" -d /tmp/shotout \
 *       $SRC/org/firstinspires/ftc/teamcode/Hardware/Shooting/{RobotState,ShotSolution,BallisticsModel,ShotSolver,ShotSolverSanity}.java \
 *       $SRC/org/firstinspires/ftc/teamcode/Util/Wrapper/InterpLUT.java
 *   java -cp "/tmp/shotout:$CP" org.firstinspires.ftc.teamcode.Hardware.Shooting.ShotSolverSanity
 * </pre>
 */
public class ShotSolverSanity {

    private static int fails = 0;

    private static void check(String what, boolean ok, String detail) {
        System.out.printf("  %-52s %s  %s%n", what, ok ? "PASS" : "FAIL", detail);
        if (!ok) fails++;
    }

    public static void main(String[] args) {
        BallisticsModel b = new BallisticsModel();
        double goalX = -70, goalY = 69, off = -1.66;

        RobotState still = new RobotState(55, 14, Math.PI / 2, 0, 0, 0, 0, 0);
        ShotSolution s0 = ShotSolver.solve(still, goalX, goalY, off, b);
        double bearing = Math.atan2(goalY - still.shooterY(off), goalX - still.shooterX(off));
        check("stationary: virtual distance == true distance",
                Math.abs(s0.virtualDistance - s0.trueDistance) < 1e-6,
                String.format("d=%.2f", s0.trueDistance));
        check("stationary: azimuth == direct bearing",
                Math.abs(ShotSolver.wrap(s0.azimuthField - bearing)) < 1e-6,
                String.format("%.3f rad", s0.azimuthField));
        check("stationary: turret rate is zero", Math.abs(s0.requiredTurretRate) < 1e-9, "");

        double dx = goalX - still.shooterX(off), dy = goalY - still.shooterY(off);
        double n = Math.hypot(dx, dy);
        double latX = -dy / n, latY = dx / n;
        double sp = 40;
        ShotSolution sl = ShotSolver.solve(
                new RobotState(55, 14, Math.PI / 2, latX * sp, latY * sp, 0, 0, 0), goalX, goalY, off, b);
        ShotSolution sr = ShotSolver.solve(
                new RobotState(55, 14, Math.PI / 2, -latX * sp, -latY * sp, 0, 0, 0), goalX, goalY, off, b);
        double leadL = ShotSolver.wrap(sl.azimuthField - bearing);
        double leadR = ShotSolver.wrap(sr.azimuthField - bearing);
        check("lateral motion produces a lead", Math.abs(leadL) > 0.01,
                String.format("%.2f deg", Math.toDegrees(leadL)));
        check("reversing motion mirrors the lead", Math.abs(leadL + leadR) < 1e-6,
                String.format("%.2f vs %.2f deg", Math.toDegrees(leadL), Math.toDegrees(leadR)));
        check("lead is opposite the motion", leadL < 0, "+lateral motion -> -azimuth");

        ShotSolution ss = ShotSolver.solve(
                new RobotState(55, 14, Math.PI / 2, 0, 0, 0, 0, 1.5), goalX, goalY, off, b);
        check("pure spin needs -omega of turret rate",
                Math.abs(ss.requiredTurretRate + 1.5) < 1e-6,
                String.format("%.3f rad/s", ss.requiredTurretRate));
        check("translation rate ~ v_lat/d",
                Math.abs(sl.requiredTurretRate - sp / sl.virtualDistance) < 0.05,
                String.format("%.3f vs %.3f", sl.requiredTurretRate, sp / sl.virtualDistance));

        ShotSolution sc = ShotSolver.solve(
                new RobotState(55, 14, Math.PI / 2, dx / n * sp, dy / n * sp, 0, 0, 0), goalX, goalY, off, b);
        check("closing reduces virtual distance", sc.virtualDistance < sc.trueDistance,
                String.format("%.1f < %.1f", sc.virtualDistance, sc.trueDistance));

        // Facing the goal from 50 inches, strafing at 60 in/s and turning: with an Axon Max MK2 at
        // 6V (522 deg/s unloaded, derated to 350) this is comfortably inside the envelope. Rate only
        // binds when a hard spin adds to the bearing drift rather than cancelling it.
        ShotSolution sok = ShotSolver.solve(
                new RobotState(goalX + 50, goalY, Math.PI, 0, 60, 0, 0, 1.0), goalX, goalY, off, b);
        check("realistic fast close shot stays feasible", sok.feasible,
                String.format("rate=%.0f deg/s", Math.toDegrees(sok.requiredTurretRate)));

        // Same geometry at 20 inches with the spin now adding to the drift instead of opposing it.
        ShotSolution sf = ShotSolver.solve(
                new RobotState(goalX + 20, goalY, Math.PI, 0, 60, 0, 0, 5.0), goalX, goalY, off, b);
        check("spin adding to the bearing drift trips the rate limit",
                !sf.feasible && "turret rate".equals(sf.reason),
                String.format("rate=%.0f deg/s, %s", Math.toDegrees(sf.requiredTurretRate), sf.reason));

        // Travel: facing directly away puts the goal 180 degrees off, past the ~140 degree clip.
        ShotSolution st = ShotSolver.solve(
                new RobotState(goalX + 50, goalY, 0, 0, 0, 0, 0, 0), goalX, goalY, off, b);
        check("azimuth beyond travel is flagged",
                !st.feasible && "turret travel".equals(st.reason),
                String.format("%.0f deg, %s", Math.toDegrees(st.azimuthRelative), st.reason));

        BallisticsModel.USE_DRAG_MODEL = true;
        double f4 = b.lateralLeadFactor(0.4), f9 = b.lateralLeadFactor(0.9);
        check("drag lead factor falls with flight time", f4 > f9 && f9 > 0 && f4 < 1,
                String.format("T=0.4 -> %.2f, T=0.9 -> %.2f", f4, f9));
        BallisticsModel.USE_DRAG_MODEL = false;

        System.out.println(fails == 0 ? "\nALL PASS" : "\n" + fails + " FAILURES");
        System.exit(fails == 0 ? 0 : 1);
    }
}

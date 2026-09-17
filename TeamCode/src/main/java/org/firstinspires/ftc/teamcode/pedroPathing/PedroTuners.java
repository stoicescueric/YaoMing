package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

/**
 * Autotune procedures for Pedro 3.
 *
 * <p>Pedro 3's autotune is a framework, not a set of ready-made tuners: {@code core}, {@code revhub}
 * and {@code tuning} ship the machinery and zero procedures. The tuners themselves are team code,
 * which is what this file is for.
 *
 * <h3>The contract</h3>
 * {@link TunerScanner} walks every loaded class for methods annotated {@link Tuner} and rejects
 * anything that is not:
 * <ul>
 *   <li>{@code static},</li>
 *   <li>zero-argument,</li>
 *   <li>returning {@link Procedure}.</li>
 * </ul>
 * It then invokes the method and hands the result to
 * {@code TunerRegistrar.register(tuner.name(), procedure)}. Discovery happens at app start through
 * Sloth's class scanner, so there is nothing to call and nothing to list: a correctly shaped method
 * simply appears in the browser UI that {@code tuning} serves.
 *
 * <p>Inside {@link Procedure#run()} the protected helpers are {@code confirmation(title, body)} to
 * wait for the operator, {@code inputs(title, body)} plus {@code awaitInputs} to collect numbers,
 * {@code withDisplay} to draw, and {@code abort(reason)} to bail out.
 *
 * <p><b>Status.</b> The contract above was read out of the shipped bytecode rather than from
 * documentation, and the shape below compiles against it. What has not been confirmed on a robot is
 * the end to end run: that Sloth discovers these at app start, that the web UI serves, and how a
 * procedure that needs to drive should launch its {@code TuningOpMode}. Verify this one works before
 * writing tuners that move the robot.
 */
public final class PedroTuners {

    private PedroTuners() { }

    /**
     * Smallest thing that exercises the whole path: register, appear in the UI, take input, confirm.
     *
     * <p>Deliberately does not touch hardware. Run it first to prove discovery and the UI work; once
     * it does, the driving tuners are worth writing.
     */
    @Tuner(name = "Pedro 3 connectivity check")
    public static Procedure connectivityCheck() {
        return new Procedure("Pedro 3 connectivity check",
                "Confirms the autotune UI can reach the robot and hand values back. Moves nothing.") {
            @Override
            public void run() throws InterruptedException {
                confirmation("Ready",
                        "If you can read this in the browser, discovery and the web server work.");

                Inputs in = inputs("Echo a number",
                        "Type any value. It is read back below, which proves the round trip.");
                Inputs.NumberField<Double> value = in.d("value");
                awaitInputs(in);

                confirmation("Round trip complete",
                        "Received: " + value.get() + ". Autotune is wired up correctly.");
            }
        };
    }

    /*
     * Where the real tuners go.
     *
     * <p>{@link Constants} currently carries four numbers straight from the Pedro 2 setup that
     * describe the drivetrain rather than the control law, so they are the ones worth measuring
     * first: {@code MAX_FORWARD_VELOCITY}, {@code MAX_STRAFE_VELOCITY},
     * {@code NATURAL_FORWARD_DECEL} and {@code NATURAL_STRAFE_DECEL}. Each needs the robot driven
     * and then released, which means a {@code TuningOpMode} launched from inside {@code run()}.
     * Left unwritten on purpose: guessing at that launch contract and shipping it untested would be
     * worse than leaving the gap visible.
     */
}

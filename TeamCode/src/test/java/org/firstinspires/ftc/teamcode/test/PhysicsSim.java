package org.firstinspires.ftc.teamcode.test;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

/**
 * Models what I have experimented FTC motors physics to be like.
 * <p>
 * <pre>
 * a = max(-slipDeceleration,
 *         currentVoltage / appliedVoltage * (maxAcceleration + naturalDeceleration)
 *          - signum(velocity) * naturalDeceleration
 *          - (if signum(velocity) != signum(appliedVoltage) ? velocity * kBackEMF : 0)
 * </pre>
 * Note: slippage is unavoidable in FTC for high speeds.
 * <p>
 * 1. Natural deceleration (+0 or positive voltage) is very slow (~-30in/s/s)
 * <p>
 * 2. Braking with negative voltage actives regenerative braking which is a strong unavoidable force that opposes motion (~-200 in/s at 60 in/s)
 * <p>
 * Goal: given curren velocity and distance remaining,
 * what motorPower should be applied to reach a desired stopping point as fast as possible?
 * <p>
 * Optional: allow velocity and acceleration limits and more than full stops
 * <p>
 */
public class PhysicsSim {
    private final double dt = 0.015; // 15ms
    private final double maxAccel = 60; // inches/s/s
    public final double zeroPowerAcceleration = 30; // -inches/s/s
    private final double requiredPowerToMove = 0.05;
    private final double slipDecel = 200;
    private final double backEMF = 2.5; // -inches/s^2 per velocity when opposing motion
    private final double maxSpeed = 80;

    private double position = 0;
    private double speed = 0;
    private double virtualTime = 0;
    private double motorPower = 0;

    public void setMotorPower(double power) {
        this.motorPower = Math.max(-1, Math.min(1, power));
    }

    public void update() throws InterruptedException {
        double accel = motorPower * (maxAccel + zeroPowerAcceleration);

        if (Math.signum(speed) != Math.signum(motorPower) && Math.abs(speed) > 0.1) {
            accel -= speed * backEMF;
        }

        if (Math.abs(speed) < 1) {
            double staticFriction = maxAccel * requiredPowerToMove;

            if (Math.abs(accel) < staticFriction) {
                accel = 0;
                speed = 0;
            } else {
                accel -= Math.signum(accel) * staticFriction;
            }
        } else {
            accel -= Math.signum(speed) * zeroPowerAcceleration;
        }

        accel = Math.max(-slipDecel, accel);

        speed += accel * dt;
        position += speed * dt;
        speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

        virtualTime += dt;
        Thread.sleep((long) (dt*1000));

        logState(accel);
    }

    public double findVelocityAtDistance(double initialVelocity, double remainingDistance, double brakePower) {
        double speed = initialVelocity;
        double position = 0;

        // Determine direction - we need to move in the sign of remainingDistance
        int direction = (int) Math.signum(remainingDistance);
        if (direction == 0) return initialVelocity; // No distance to travel

        double targetDistance = Math.abs(remainingDistance);
        double maxIterations = 1000000; // Prevent infinite loops
        int iterations = 0;

        double accel=0;

        // Simulate until we reach the target distance or exceed max iterations
        while (Math.abs(position) < targetDistance && iterations < maxIterations) {
            // Calculate acceleration using the physics model from update()
            accel = brakePower * (maxAccel + zeroPowerAcceleration);

            // Back EMF when changing direction
            if (Math.signum(speed) != Math.signum(brakePower) && Math.abs(speed) > 0.1) {
                accel -= speed * backEMF;
            }

            // Static friction at low speeds
            if (Math.abs(speed) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;

                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                    speed = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }
            } else {
                // Kinetic friction
                accel -= Math.signum(speed) * zeroPowerAcceleration;
            }

            // Apply slip deceleration limit
            accel = Math.max(-slipDecel, accel);

            // Update velocity and position
            speed += accel * dt;
            position += speed * dt;

            // Apply speed limit
            speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

            iterations++;
        }

        // If we've stopped before reaching the target
        if (Math.abs(speed) == 0 && Math.abs(position) < targetDistance) {
            return calculateReverseDecelerationVelocity(initialVelocity, targetDistance);
        }

        // If we've overshot, estimate the velocity at the target using linear interpolation
        if (Math.abs(position) >= targetDistance) {
            // Simple linear interpolation back to target
            double lastPos = position - speed * dt;
            if (Math.abs(lastPos - targetDistance) < 1e-6) {
                return speed;
            }

            // Interpolate: how far past the target did we go?
            double overshoot = Math.abs(position) - targetDistance;
            double fracOfStep = 1.0 - (overshoot / Math.abs(speed * dt));
            fracOfStep = Math.max(0, Math.min(1, fracOfStep));

            // Estimate velocity at target (assumes roughly linear velocity change over the step)
            double prevSpeed = speed - (accel * dt);
            return prevSpeed + (speed - prevSpeed) * fracOfStep;
        }

        return speed;
    }

    /**
     * Calculates what velocity would be needed if we reversed the deceleration process.
     * This represents the "negative velocity" for unreachable targets.
     */
    private double calculateReverseDecelerationVelocity(double initialVelocity, double targetDistance) {
        // Simulate backwards: start from target, go backwards with deceleration
        double speed = 0;
        double position = 0;
        int iterations = 0;

        while (Math.abs(position) < targetDistance && iterations < 1000000) {
            // Reverse deceleration: we're moving opposite to initial velocity
            double accel = -motorPower * (maxAccel + zeroPowerAcceleration);

            // Back EMF when changing direction
            if (Math.signum(speed) != Math.signum(-motorPower) && Math.abs(speed) > 0.1) {
                accel -= speed * backEMF;
            }

            // Static friction at low speeds
            if (Math.abs(speed) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;
                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                    speed = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }
            } else {
                accel -= Math.signum(speed) * zeroPowerAcceleration;
            }

            accel = Math.max(-slipDecel, accel);
            speed += accel * dt;
            position += speed * dt;
            speed = Math.max(-maxSpeed, Math.min(maxSpeed, speed));

            iterations++;
        }

        // Return negative to indicate this is the "reverse" velocity
        return -speed;
    }


    double predictStoppingDistance(double velocity, double motorPower) {
        double v = velocity;
        double dist = 0.0;

        for (int i = 0; i < 5000; i++) {

            if (Math.abs(v) < 0.001) break;

            double accel = motorPower * (maxAccel + zeroPowerAcceleration);

            // back EMF (same logic as update)
            if (Math.signum(v) != Math.signum(motorPower) && Math.abs(v) > 0.1 || motorPower == 0) {
                accel -= v * backEMF;
            }

             if (Math.abs(v) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;

                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }

            } else {
                accel -= Math.signum(v) * zeroPowerAcceleration;
            }

            accel = Math.max(-slipDecel, accel);

            double nextV = v + accel * dt;

            if (Math.signum(nextV) != Math.signum(v)) {
                dist += 0.5 * v * dt; // better than full-step approximation
                break;
            }

            dist += 0.5 * (v + nextV) * dt; // trapezoidal integration
            v = nextV;
        }

        return dist;
    }

    double excessVelocityAfterBraking(
            double availableDisplacement,
            double brakingDisplacement,
            double brakingPower
    ) {
        double overshootDisplacement =
                brakingDisplacement - availableDisplacement;

        boolean stopsBeforeTarget =
                Math.signum(overshootDisplacement)
                        != Math.signum(availableDisplacement);

        if (stopsBeforeTarget) {
            return 0;
        }

        return maxVelocityToStopWithinDistance(overshootDisplacement, brakingPower);
    }

    public double finalVelocityAtDistance2(double initialVelocity, double motorPower, double targetDistance) {

        double v = initialVelocity;
        double pos = 0.0;

        double direction = Math.signum(targetDistance - pos);
        if (direction == 0) return v;

        for (int i = 0; i < 5000; i++) {

            double remaining = targetDistance - pos;

            // stop condition: we crossed the target in correct direction
            if (Math.signum(remaining) != direction || Math.abs(v) < 0.001) {
                return v;
            }

            double accel = motorPower * (maxAccel + zeroPowerAcceleration);

            // back EMF
            if ((Math.signum(v) != Math.signum(motorPower) && Math.abs(v) > 0.1) || motorPower == 0) {
                accel -= v * backEMF;
            }

            // friction model
            if (Math.abs(v) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;

                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }
            } else {
                accel -= Math.signum(v) * zeroPowerAcceleration;
            }

            accel = Math.max(-slipDecel, accel);

            double nextV = v + accel * dt;

            double stepPos = 0.5 * (v + nextV) * dt;
            pos += stepPos;

            v = nextV;
        }

        return v;
    }

    public double finalVelocityAtDistance(double initialVelocity, double motorPower, double targetDistance) {
        double v = initialVelocity;
        double dist = 0.0;

        // Loop until the distance traveled reaches targetDistance
        while (dist < Math.abs(targetDistance)) {
            // Calculate acceleration
            double accel = motorPower * (maxAccel + zeroPowerAcceleration);

            // Back EMF effect
            if (Math.signum(v) != Math.signum(motorPower) && Math.abs(v) > 0.1 || motorPower == 0) {
                accel -= v * backEMF;
            }

            // Friction and static friction handling
            if (Math.abs(v) < 1) {
                double staticFriction = maxAccel * requiredPowerToMove;
                if (Math.abs(accel) < staticFriction) {
                    accel = 0;
                } else {
                    accel -= Math.signum(accel) * staticFriction;
                }
            } else {
                accel -= Math.signum(v) * zeroPowerAcceleration;
            }

            // Limit acceleration
            accel = Math.max(-slipDecel, Math.min(accel, slipDecel));

            // Update velocity
            double nextV = v + accel * dt;

            // Handle sign change (stopping or reversing)
            if (Math.signum(nextV) != Math.signum(v) && Math.abs(nextV) < 0.001) {
                // If velocity crosses zero, stop
                dist += 0.5 * v * dt; // approximate distance during zero crossing
                v = 0;
                break;
            }

            // Update distance with trapezoidal integration
            dist += 0.5 * (v + nextV) * dt;
            v = nextV;

            // Safety: break if simulation runs too long
            if (dist > 1e6) break;
        }

        // Return the final velocity, preserving sign
        return v;
    }

    public double bangBang(double currentVelocity, double distanceRemaining) {
        double dir = Math.signum(distanceRemaining);
        double stoppingDist = predictStoppingDistance(currentVelocity, -dir);

        if (Math.signum(stoppingDist) == Math.signum(distanceRemaining)
                && Math.abs(stoppingDist) + 0.3 >= Math.abs(distanceRemaining)) {
            return -dir; // brake
        } else {
            return dir;  // accelerate
        }
    }

    public double minimumPowerToMaxDecel(double currentVelocity, double distanceRemaining) {
        // required decel to stop exactly at target
        double desiredDecel =
                (currentVelocity * currentVelocity)
                        / (2.0 * Math.abs(distanceRemaining));

        desiredDecel =
                Math.min(desiredDecel, slipDecel);

        // passive braking already happening
        double passiveDecel =
                zeroPowerAcceleration
                        + Math.abs(currentVelocity) * backEMF;

        double motorDecel =
                Math.max(0, desiredDecel - passiveDecel);

        double power =
                motorDecel
                        / (maxAccel + zeroPowerAcceleration);

        // opposite velocity
        power *= -Math.signum(currentVelocity);

        return Math.max(-1, Math.min(1, power));
    }

    public double minimumBrakePowerBangBang(double currentVelocity,
                                            double distanceRemaining) {

        double dir = Math.signum(distanceRemaining);

        double stoppingDist = predictStoppingDistance(currentVelocity, -dir);

        if (Math.signum(stoppingDist) != Math.signum(distanceRemaining)
                || Math.abs(stoppingDist) + currentVelocity * dt < Math.abs(distanceRemaining)) {

            // accelerate toward target
            return dir;

        } else {
            return minimumPowerToMaxDecel(currentVelocity, distanceRemaining);
        }
    }


    public double stoppingDistance(double v, double brakingPower) {
        double sign = Math.signum(v);
        v = Math.abs(v);
        if (v < 1e-9) return 0.0;

        double k   = backEMF;
        double zPA = zeroPowerAcceleration;
        double sd  = slipDecel;
        double C   = zPA + (-brakingPower * (maxAccel + zPA));

        double totalDist = 0.0;

        // ── Phase 1: slip ────────────────────────────────────────────────────
        double vExit = (Math.abs(k) > 1e-9) ? Math.max((sd - C) / k, 0.0) : Double.MAX_VALUE;

        if (v > vExit) {
            totalDist += (v * v - vExit * vExit) / (2.0 * sd);
            v = vExit;
        }

        if (v < 1e-9) return sign * totalDist;

        // ── Phase 2: drag ODE  dv/dt = -(C + k*v) ───────────────────────────
        if (Math.abs(k) < 1e-6) {
            totalDist += (v * v) / (2.0 * C);
        } else {
            totalDist += v / k - (C / (k * k)) * Math.log1p(k * v / C);
        }

        Logger.recordOutput("stoppingDistance", sign * totalDist);

        return sign * totalDist;
    }

//    public double finalVelocityAfterBraking(double initialVelocity,
//                                            double distance,
//                                            double brakingPower) {
//
//        double sign = Math.signum(initialVelocity);
//        double v0 = Math.abs(initialVelocity);
//        double absD = Math.abs(distance);
//
//        if (v0 < 1e-9 || absD <= 0.0)
//            return initialVelocity;
//
//        // If we stop before reaching the distance, return 0
//        double stopDist = Math.abs(predictStoppingDistance(initialVelocity, brakingPower));
//        if (stopDist <= absD)
//            return 0.0;
//
//        double vLo = 0.0;
//        double vHi = v0;
//
//        // Binary search for velocity whose stopping distance equals remaining distance
//        // stoppingDistance(vf) = stoppingDistance(v0) - distance
//        double targetRemaining = stopDist - absD;
//
//        for (int i = 0; i < 64; i++) {
//            double vMid = (vLo + vHi) * 0.5;
//
//            double dMid = Math.abs(predictStoppingDistance(vMid, brakingPower));
//
//            if (dMid > targetRemaining)
//                vHi = vMid;
//            else
//                vLo = vMid;
//
//            if ((vHi - vLo) / (vHi + 1e-12) < 1e-6)
//                break;
//        }
//
//        return ((vLo + vHi) * 0.5) * sign;
//    }

    public double stoppingDistanceQuadratic(double v, double brakingPower) {
        int N = 40;
        double maxV = maxSpeed;

        double sumV2 = 0, sumV3 = 0, sumV4 = 0, sumV2d = 0, sumVd = 0;
        double[] vs = new double[N], ds = new double[N];
        for (int i = 0; i < N; i++) {
            double vi = maxV * (i + 1) / N;
            double di = stoppingDistance(vi, brakingPower);
            vs[i] = vi;
            ds[i] = di;
            sumV2  += vi * vi;
            sumV3  += vi * vi * vi;
            sumV4  += vi * vi * vi * vi;
            sumV2d += vi * vi * di;
            sumVd  += vi * di;
        }

        double det = sumV4 * sumV2 - sumV3 * sumV3;
        double a   = (sumV2d * sumV2 - sumVd  * sumV3) / det;
        double b   = (sumV4  * sumVd  - sumV3 * sumV2d) / det;

        // R² and mean absolute error
        double dMean = 0;
        for (double di : ds) dMean += di;
        dMean /= N;

        double ssTot = 0, ssRes = 0, sumAbsErr = 0;
        for (int i = 0; i < N; i++) {
            double predicted = a * vs[i] * vs[i] + b * vs[i];
            double residual  = ds[i] - predicted;
            ssTot    += (ds[i] - dMean) * (ds[i] - dMean);
            ssRes    += residual * residual;
            sumAbsErr += Math.abs(residual);
        }
        double r2  = 1.0 - ssRes / ssTot;
        double mae = sumAbsErr / N;

        Logger.recordOutput("StoppingDist/brakingPower", brakingPower);
        Logger.recordOutput("StoppingDist/a",            a);
        Logger.recordOutput("StoppingDist/b",            b);
        Logger.recordOutput("StoppingDist/R2",           r2);
        Logger.recordOutput("StoppingDist/MAE",          mae);

        double sign = Math.signum(v);
        double vAbs = Math.abs(v);
        return sign * (a * vAbs * vAbs + b * vAbs);
    }

    public double maxVelocityToStopWithinDistance(double distance, double brakingPower) {
        if (distance == 0) return 0.0;

        // Remember the sign of distance
        int sign = distance < 0 ? -1 : 1;
        distance = Math.abs(distance);

        double lo = 0.0;

        // safe upper bound guess (grow until we exceed distance)
        double hi = 1.0;
        while (stoppingDistance(hi, brakingPower) < distance) {
            hi *= 2.0;
            if (hi > 100) break; // safety cap for simulation
        }

        // binary search
        for (int i = 0; i < 40; i++) {
            double mid = 0.5 * (lo + hi);

            double d = stoppingDistance(mid, brakingPower);

            if (d > distance) {
                hi = mid;   // too fast to stop in time
            } else {
                lo = mid;   // can still fit, try faster
            }
        }

        return sign * lo;
    }

    private void logState(double accel) {
        Logger.recordOutput("sim/power", motorPower);
        Logger.recordOutput("sim/speed", speed);
        Logger.recordOutput("sim/position", position);
        Logger.recordOutput("sim/accel", accel);

        double inchesToMeters = 0.0254;
        Logger.recordOutput("sim/pose", new Pose2d(position * inchesToMeters, 0, new Rotation2d(0)));
    }

    public double getPosition() { return position; }
    public double getSpeed() { return speed; }
    public double getVirtualTime() { return virtualTime; }
    public double getDt() { return dt; }
    
    public void reset() {
        position = 0;
        speed = 0;
        virtualTime = 0;
        motorPower = 0;
    }
}

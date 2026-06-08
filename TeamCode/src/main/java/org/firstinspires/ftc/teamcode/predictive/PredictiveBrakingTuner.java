//package org.firstinspires.ftc.teamcode;
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
//import org.firstinspires.ftc.teamcode.blackice.core.Follower;
//import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackice.utils.MathUtils;
//import org.firstinspires.ftc.teamcode.blackice.utils.Regression;
//
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.List;
//import java.util.function.Predicate;
//
//@Autonomous
//@Config
//public class PredictiveBrakingTuner extends OpMode {
//    public static BrakingOptions brakingOptions = new BrakingOptions();
//
//    private static double[] DRIVE_POWERS;
//
//    private final ElapsedTime timer = new ElapsedTime();
//    private State state = State.START_MOVE;
//    private int iteration = 0;
//
//    private Follower follower;
//
//    private final List<BrakeRun> brakeRuns = new ArrayList<>();
//    private BrakeRun currentRun;
//
//    @Override
//    public void init() {
//        DRIVE_POWERS = MathUtils.biasedGradient(brakingOptions.trials, brakingOptions.max, brakingOptions.min, brakingOptions.highPowerBias);
//
//        follower = FollowerConstants.createFollower(hardwareMap);
//
//        follower.setTelemetry(telemetry);
//
//        follower.update();
//
//        if (brakingOptions.zeroPowerBrakeMode) {
//            follower.drivetrain.zeroPowerBrakeMode();
//        } else {
//            follower.drivetrain.zeroPowerFloatMode();
//        }
//    }
//
//    @Override
//    public void start() {
//        timer.reset();
//        follower.update();
//    }
//
//    // amount of velocity that is in the direction of the robot's forward acceleration axis
//    public double getRobotVelocityTowardAccel() {
//        return follower.getVelocity().dot(new Vector(getDirection(), follower.getHeading() + brakingOptions.theta));
//    }
//
//    // amount of velocity that is in the direction of the acceleration axis (regardless of robot heading)
//    public double getFieldVelocityTowardAccel() {
//        return follower.getVelocity().dot(getDriveDirection());
//    }
//
//    // speed the wheels are moving toward the acceleration direction
//    public double averageWheelVelocity() {
//        return follower.getAverageWheelVelocity(brakingOptions.theta);
//    }
//
//    public double getVelocityMagnitude() {
//        return follower.getVelocity().computeMagnitude();
//    }
//
//    private int getDirection() {
//        return (iteration % 2 == 0) ? 1 : -1;
//    }
//
//    private void drive(double power) {
//        double turnPower = 0;
//        if (brakingOptions.correctHeadingWhileAccel) {
//            turnPower = follower.computeHeadingCorrectionPower(0);
//        }
//
//        follower.drivetrain.followVector(getDriveDirection().times(power), turnPower);
//    }
//
//    private void startBraking() {
//        currentRun = new BrakeRun();
//        brakeRuns.add(currentRun);
//        currentRun.add(getBrakingSnapshot());
//
//        double turnPower = 0;
//        if (brakingOptions.correctHeadingWhileBrake) {
//            turnPower = follower.computeHeadingCorrectionPower(0);
//        }
//
//        follower.drivetrain.followVector(
//                getDriveDirection().times(-brakingOptions.brakingPower), turnPower);
//        timer.reset();
//    }
//
//    private BrakingSnapshot getBrakingSnapshot() {
//        double t = timer.milliseconds();
//        Vector position = follower.getPosition();
//        double headingError = -follower.getHeading();
//
//        double velocityMagnitude = getVelocityMagnitude();
//        double fieldVelocity = getFieldVelocityTowardAccel();
//        double robotVelocity = getRobotVelocityTowardAccel();
//        double averageWheelVelocity = averageWheelVelocity();
//        double velocitySlippage = robotVelocity - averageWheelVelocity;
//
//        double voltage = follower.getVoltage();
//        double appliedVoltage = voltage * -brakingOptions.brakingPower;
//
//        return new BrakingSnapshot(t, position, headingError, robotVelocity, velocityMagnitude,
//                                fieldVelocity, robotVelocity, voltage, appliedVoltage, averageWheelVelocity, velocitySlippage);
//    }
//
//    public Vector getDriveDirection() {
//        return Vector.fromPolar(getDirection(), brakingOptions.theta);
//    }
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void loop() {
//
//        follower.update();
//
//        if (gamepad1.b) {
//            follower.drivetrain.zeroPower();
//            follower.drivetrain.zeroPowerBrakeMode();
//            requestOpModeStop();
//            return;
//        }
//
//        switch (state) {
//            case ACCEL:
//                if (timer.milliseconds() >= brakingOptions.ACCEL_TIME_MS) {
//                    startBraking();
//
//                    timer.reset();
//                    state = State.BRAKE;
//                    break;
//                }
//                drive(DRIVE_POWERS[iteration]);
//                break;
//            case BRAKE:
//                BrakingSnapshot snapshot = getBrakingSnapshot();
//                currentRun.add(snapshot);
//
//                if (snapshot.robotVelocity <= brakingOptions.velocityThreshold) {
//                    iteration++;
//
//                    if (iteration >= DRIVE_POWERS.length) {
//                        state = State.DONE;
//                    } else {
//                        drive(DRIVE_POWERS[iteration]);
//                        state = State.ACCEL;
//                    }
//                    return;
//                }
//
//                double turnPower = 0;
//                if (brakingOptions.correctHeadingWhileBrake) {
//                    turnPower = follower.computeHeadingCorrectionPower(0);
//                }
//
//                follower.drivetrain.followVector(
//                        getDriveDirection().times(-brakingOptions.brakingPower), turnPower);
//                break;
//            case DONE:
//                follower.drivetrain.zeroPower();
//                follower.drivetrain.zeroPowerBrakeMode();
//
//                follower.telemetry.addLine("Tuning Complete");
//
//                follower.telemetry.addLine("Initial Braking Profile:");
//                quadFitRuns(brakeRuns, i -> i == 0);
//
//                follower.telemetry.addLine("Total Braking Profile");
//                quadFitRuns(brakeRuns, i -> true);
//
//                follower.telemetry.addLine("Non-Initial Braking Profile:");
//                quadFitRuns(brakeRuns, i -> i > 0);
//
//                follower.telemetry.update();
//
//                // Export CSV data
//                exportVelocityToDistanceCSV();
//                exportBrakeDataCSV();
//
//                requestOpModeStop();
//                break;
//        }
//    }
//
//    public static class BrakeRun {
//        List<BrakingSnapshot> records = new ArrayList<>();;
//
//        public void add(BrakingSnapshot brakeSnapshot) {
//            records.add(brakeSnapshot);
//        }
//
//        List<double[]> toRemainingVelocityDistance(Predicate<Integer> filter) {
//            List<double[]> out = new ArrayList<>();
//            if (records.size() < 2) return out;
//
//            Vector finalPos = records.get(records.size() - 1).position;
//
//            for (int i = 0; i < records.size() - 1; i++) {
//                if (!filter.test(i)) continue;
//
//                BrakingSnapshot r = records.get(i);
//
//                double d = finalPos.minus(r.position).computeMagnitude();
//                out.add(new double[]{r.velocity, d});
//            }
//
//            return out;
//        }
//    }
//
//    public void quadFitRuns(
//            List<BrakeRun> runs,
//            Predicate<Integer> filter) {
//
//        List<double[]> data = new ArrayList<>();
//
//        for (BrakeRun run : runs) {
//            data.addAll(run.toRemainingVelocityDistance(filter));
//        }
//
//        double[] coefficients = Regression.quadraticFit(data);
//
//        follower.telemetry.addLine(
//                "quadratic " + String.format("%.4f", coefficients[1]) +
//                        "\nlinear " + String.format("%.3f", coefficients[0]) +
//                        "\nr² " + String.format("%.4f", coefficients[2]) +
//                        "\navg error " + String.format("%.3f", coefficients[3])
//        );
//    }
//
//    @Override
//    public void stop() {
//        follower.drivetrain.zeroPower();
//        follower.drivetrain.zeroPowerBrakeMode();
//    }
//
//    @SuppressLint("DefaultLocale")
//    private void exportVelocityToDistanceCSV() {
//        try (FileWriter writer = new FileWriter("/sdcard/FIRST/velocity_to_distance.csv")) {
//
//            writer.write("type,velocity,distance\n");
//
//            for (BrakeRun run : brakeRuns) {
//                List<BrakingSnapshot> records = run.records;
//                if (records.size() < 2) continue;
//
//                Vector lastPosition = records.get(records.size() - 1).position;
//
//                for (int i = 0; i < records.size() - 1; i++) {
//                    BrakingSnapshot r = records.get(i);
//
//                    double remainingDistance =
//                            lastPosition.minus(r.position).computeMagnitude();
//
//                    writer.write(String.format(
//                            "remaining,%.6f,%.6f\n",
//                            r.robotVelocity,
//                            remainingDistance
//                    ));
//                }
//            }
//
//            follower.telemetry.addLine("Exported velocity_to_distance.csv");
//
//        } catch (IOException e) {
//            follower.telemetry.addLine("Failed velocity CSV: " + e.getMessage());
//        }
//    }
//
//    @SuppressLint("DefaultLocale")
//    private void exportBrakeDataCSV() {
//        try (FileWriter writer = new FileWriter("/sdcard/FIRST/brake_data.csv")) {
//
//            writer.write(
//                    "timeMs,x,y,headingError,velocity,velocityMagnitude,fieldVelocity,robotVelocity,voltage,voltageApplied\n"
//            );
//
//            for (BrakeRun run : brakeRuns) {
//                for (BrakingSnapshot r : run.records) {
//                    writer.write(String.format(
//                            "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
//                            r.timeMs,
//                            r.position.getX(),
//                            r.position.getY(),
//                            r.headingError,
//                            r.robotVelocity,
//                            r.velocityMagnitude,
//                            r.fieldVelocity,
//                            r.robotVelocity,
//                            r.voltage,
//                            r.voltageApplied
//                    ));
//                }
//            }
//
//            follower.telemetry.addLine("Exported brake_data.csv");
//
//        } catch (IOException e) {
//            follower.telemetry.addLine("Failed brake CSV: " + e.getMessage());
//        }
//    }
//
//    private enum State {
//        START_MOVE,
//        ACCEL,
//        BRAKE,
//        DONE
//    }
//}
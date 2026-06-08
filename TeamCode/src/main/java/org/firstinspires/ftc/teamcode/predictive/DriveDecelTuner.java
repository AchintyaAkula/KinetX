//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.psilynx.psikit.core.AutoLog;
//
//@AutoLog
//@Autonomous
//public class DriveDecelTuner extends OpMode {
//    private DcMotor fr;
//    private DcMotor fl;
//    private DcMotor br;
//    private DcMotor bl;
//    private IMU imu;
//
//    private double targetHeading = 0;
//
//    @Override
//    public void init() {
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//
//        fl.setDirection(DcMotor.Direction.REVERSE);
//        bl.setDirection(DcMotor.Direction.REVERSE);
//
//        // Reset encoders and set to run without encoder for simple power control
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust these to match your actual hub orientation
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//    }
//
//    @Override
//    public void loop() {
//        // Example: Drive forward at 0.5 power while A is held
//        double drivePower = gamepad1.a ? 0.5 : 0;
//
//        driveForward(drivePower, targetHeading);
//
//        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Target Heading", targetHeading);
//    }
//
//    /**
//     * Drives the robot forward with heading correction.
//     * @param power The base forward power.
//     * @param targetHeading The heading to maintain (in degrees).
//     */
//    public void driveForward(double power, double targetHeading) {
//        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double headingError = targetHeading - currentHeading;
//
//        // Normalize heading error to be within [-180, 180]
//        while (headingError > 180) headingError -= 360;
//        while (headingError <= -180) headingError += 360;
//
//        // Simple proportional correction
//        double kP = 0.05;
//        double turnPower = headingError * kP;
//
//        setAllMotorPowers(power - turnPower, power + turnPower, power - turnPower, power + turnPower);
//    }
//
//    /**
//     * Sets power to all four drivetrain motors.
//     */
//    public void setAllMotorPowers(double flPower, double frPower, double blPower, double brPower) {
//        fl.setPower(flPower);
//        fr.setPower(frPower);
//        bl.setPower(blPower);
//        br.setPower(brPower);
//    }
//}

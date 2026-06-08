//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
//
//public class BrakingSnapshot {
//    double timeMs;
//    Vector position;
//    double headingError;
//    double velocity;
//    double velocityMagnitude;
//    double fieldVelocity;
//    double robotVelocity;
//    double voltage;
//    double voltageApplied;
//    double averageWheelVelocity;
//    double velocitySlippage;
//
//    BrakingSnapshot(double timeMs, Vector position, double headingError, double velocity,
//                    double velocityMagnitude, double fieldVelocity, double robotVelocity,
//                    double voltage, double voltageApplied, double averageWheelVelocity, double velocitySlippage) {
//        this.timeMs = timeMs;
//        this.position = position;
//        this.headingError = headingError;
//        this.velocity = velocity;
//        this.velocityMagnitude = velocityMagnitude;
//        this.fieldVelocity = fieldVelocity;
//        this.robotVelocity = robotVelocity;
//        this.voltage = voltage;
//        this.voltageApplied = voltageApplied;
//        this.averageWheelVelocity = averageWheelVelocity;
//        this.velocitySlippage = velocitySlippage;
//    }
//
//    public String toString() {
//        return String.format("t=%.0f ms, x=%.2f, y=%.2f, headingErr=%.2f, " +
//                                 "vel=%.2f, velMag=%.2f, fieldVel=%.2f, robotVel=%.2f, " +
//                                 "voltage=%.2f, voltageApplied=%.2f",
//                             timeMs, position.getX(), position.getY(), headingError,
//                             velocity, velocityMagnitude, fieldVelocity, robotVelocity,
//                             voltage, voltageApplied);
//    }
//}
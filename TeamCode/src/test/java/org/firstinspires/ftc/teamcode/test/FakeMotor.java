package org.firstinspires.ftc.teamcode.test;

public class FakeMotor {
    private final PhysicsSim sim;
    private double power = 0;

    public FakeMotor(PhysicsSim sim) {
        this.sim = sim;
    }

    public void setPower(double power) {
        this.power = power;
        sim.setMotorPower(power);
    }

    public double getPower() {
        return power;
    }
}

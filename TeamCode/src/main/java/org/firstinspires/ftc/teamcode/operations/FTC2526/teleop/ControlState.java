package org.firstinspires.ftc.teamcode.operations.FTC2526.teleop;

public class ControlState {
    public double DRIVE;
    public double STRAFE;
    public double TWIST;
    public boolean HALF_SPEED;
    public boolean FULL_SPEED;
    public double GAS;
    public double BRAKE;
    public boolean DPAD_STRAFE_LEFT;
    public boolean DPAD_STRAFE_RIGHT;

    public boolean LOAD_AND_SHOOT;
    public boolean INCREASE_SHOOTCOUNT;
    public boolean DECREASE_SHOOTCOUNT;
    public boolean SHOOT;
    public boolean LOAD;
    public boolean PUSHER;

    public double OPERATOR_DRIVE;
    public double OPERATOR_STRAFE;
    public double OPERATOR_TWIST;
    public boolean CANCEL_STATE;
    public boolean FLYWHEEL_ON;
    public boolean FLYWHEEL_OFF;
    public boolean INCREASE_FLYWHEEL_SPEED;
    public boolean DECREASE_FLYWHEEL_SPEED;

    private ControlState() {} // private constructor

    public static class Builder {
        private final ControlState state = new ControlState();

        public Builder drive(double input) { state.DRIVE = input; return this; }
        public Builder strafe(double input) { state.STRAFE = input; return this; }
        public Builder twist(double input) { state.TWIST = input; return this; }

        public Builder halfSpeed(boolean input) { state.HALF_SPEED = input; return this; }
        public Builder fullSpeed(boolean input) { state.FULL_SPEED = input; return this; }

        public Builder gas(double input) { state.GAS = input; return this; }
        public Builder brake(double input) { state.BRAKE = input; return this; }

        public Builder dpadLeft(boolean input) { state.DPAD_STRAFE_LEFT = input; return this; }
        public Builder dpadRight(boolean input) { state.DPAD_STRAFE_RIGHT = input; return this; }

        public Builder loadAndShoot(boolean input) { state.LOAD_AND_SHOOT = input; return this; }
        public Builder increaseShootcount(boolean input) { state.INCREASE_SHOOTCOUNT = input; return this; }
        public Builder decreaseShootcount(boolean input) { state.DECREASE_SHOOTCOUNT = input; return this; }
        public Builder cancelState(boolean input) { state.CANCEL_STATE = input; return this; }

        public Builder operatorDrive(double input) { state.OPERATOR_DRIVE = input; return this; }
        public Builder operatorStrafe(double input) { state.OPERATOR_STRAFE = input; return this; }
        public Builder operatorTwist(double input) { state.OPERATOR_TWIST = input; return this; }

        public Builder flywheelOn(boolean input) { state.FLYWHEEL_ON = input; return this; }
        public Builder flywheelOff(boolean input) { state.FLYWHEEL_OFF = input; return this; }
        public Builder increaseFlywheelSpeed(boolean input) { state.INCREASE_FLYWHEEL_SPEED = input; return this; }
        public Builder decreaseFlywheelSpeed(boolean input) { state.DECREASE_FLYWHEEL_SPEED = input; return this; }

        public ControlState build() { return state; }
    }
}

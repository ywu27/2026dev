#include "SwerveModule.h"
#include <string>

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID, int cancoderID) : steerMotor(rev::spark::SparkMax(steerMotorID, rev::spark::SparkMax::MotorType::kBrushless)),
                                                                                 driveMotor(TalonFXMotor(driveMotorID)),
                                                                                 steerEnc(CAN_Coder(cancoderID)),
                                                                                 steerCTR(frc::PIDController(steerP, steerI, steerD)) {
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors() {
    // Resetting Motor settings, Encoders, putting it in brake mode
    steerMotor.ClearFaults();

    config.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    config.closedLoop.P(0.3);
    config.closedLoop.I(0);
    config.closedLoop.D(0);

    config.Inverted(true);

    // Makes motor stiff(coast mode lets it run freely)
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    // Keep the motor limit at under 20A
    config.SmartCurrentLimit(maxSteerCurrent);

    // Setpoints to initial encoder positions/speeds
    steerAngleSetpoint = steerEnc.getAbsolutePosition().getRadians();
    driveVelocitySetpoint = 0.0;

    // Set PID values for REV Drive PID
    steerCTR.EnableContinuousInput(0, 2 * PI);
    steerCTR.Reset();

    driveMotor.init();

    steerMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

float SwerveModule::getSteerAngleSetpoint() {
    return steerAngleSetpoint;
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 * Not tested
 */
void SwerveModule::setSteerAngleSetpoint(float setpt) {
    steerAngleSetpoint = setpt;
}

void SwerveModule::setDrivePositionSetpoint(float setpt) {
    drivePositionSetpoint = setpt;
    driveMode = POSITION;
}

/**
 * Set the drive motor velocity setpoint to input RPM
 * Max RPM is 5700
 */
void SwerveModule::setDriveVelocitySetpoint(float setpt) {
    driveVelocitySetpoint = setpt;
    driveMode = VELOCITY;
}

/**
 * speedFPS attribute should be in RPM
 * Sets Drive Velocity & Steer Angle
 */
void SwerveModule::setModuleState(SwerveModuleState setpt, bool takeShortestPath) {
    if (takeShortestPath) {
        SwerveModuleState outputs = moduleSetpointGenerator(getModuleState(), setpt);
        setDriveVelocitySetpoint(outputs.getSpeedFPS());
        setSteerAngleSetpoint(outputs.getRot2d().getRadians());
        prevSetpoint.setRot2d(outputs.getRot2d());
        prevSetpoint.setSpeedFPS(outputs.getSpeedFPS());
    } else {
        setDriveVelocitySetpoint(setpt.getSpeedFPS());
        setSteerAngleSetpoint(setpt.getRot2d().getRadians());
        prevSetpoint.setRot2d(setpt.getRot2d());
        prevSetpoint.setSpeedFPS(setpt.getSpeedFPS());
    }
}

SwerveModuleState SwerveModule::moduleSetpointGenerator(SwerveModuleState currState, SwerveModuleState desiredSetpoint) {
    double currAngle = currState.getRot2d().getRadians();
    // double currVel = currState.getSpeedFPS();
    double desAngle = desiredSetpoint.getRot2d().getRadians();
    double desVel = desiredSetpoint.getSpeedFPS();

    // double limitVel = ControlUtil::limitAcceleration(currVel, desVel, maxDriveAccelerationRPM, loopTime);
    // desVel = limitVel;

    double dist = fabs(currAngle - desAngle);
    bool flip = (dist > PI_2) && (((PI * 2) - dist) > PI_2);

    double setpointAngle;
    double setpointVel;
    if (flip) {
        setpointAngle = Rotation2d::radiansBound(desAngle + PI);

        double angleDist = std::min(fabs(setpointAngle - currAngle), (PI * 2) - fabs(setpointAngle - currAngle));

        // setpointVel = -(desVel * (-angleDist / PI_2) + desVel);
        setpointVel = -ControlUtil::scaleSwerveVelocity(desVel, angleDist, false);
    } else {
        setpointAngle = desAngle;

        double angleDist = std::min(fabs(setpointAngle - currAngle), (PI * 2) - fabs(setpointAngle - currAngle));

        // setpointVel = desVel * (-angleDist / PI_2) + desVel;
        setpointVel = ControlUtil::scaleSwerveVelocity(desVel, angleDist, false);
    }
    return SwerveModuleState(setpointVel, Rotation2d(setpointAngle));
}

SwerveModuleState SwerveModule::getModuleState() {
    double vel = getDriveEncoderVel();
    double angle = steerEnc.getAbsolutePosition().getRadians();

    return SwerveModuleState(vel, angle);
}

frc::SwerveModulePosition SwerveModule::getModulePosition() {
    frc::SwerveModulePosition pos;

    pos.angle = frc::Rotation2d(units::radian_t(Rotation2d::polarToCompass(getSteerEncoder().getRadians())));

    // changes from encoder rotations from feet to meters
    pos.distance = units::meter_t(getDriveEncoderPos() * moduleDriveRatio * wheelCircumFeet / 3.281);

    return pos;
}

bool SwerveModule::isFinished(float percentageBound) {
    if (driveMode == POSITION) {
        double pos = driveMotor.getPosition();
        return (pos < (drivePositionSetpoint * (1 + percentageBound))) && (pos > (drivePositionSetpoint * (1 - percentageBound)));
    } else {
        double pos = driveMotor.getVelocity();
        return (pos < (driveVelocitySetpoint * (1 + percentageBound))) && (pos > (driveVelocitySetpoint * (1 - percentageBound)));
    }
}

/**
 * This function is meant to run in a while loop
 * when moduleInhibit is true, motors are stopped
 * when moduleInhibit is false, motor PIDs are running
 * steerPID uses frc::PIDController
 * drivePID uses rev::PIDController
 */
void SwerveModule::run() {
    // Steer PID
    if (moduleInhibit) { // Thread is in standby mode
        currentSteerOutput = 0.0;
        steerMotor.StopMotor();
        driveMotor.set(TalonFXMotor::VELOCITY, 0.0);
    } else {

        double newSteerOutput = steerCTR.Calculate(steerEnc.getAbsolutePosition().getRadians(), steerAngleSetpoint);
        if (!ControlUtil::epsilonEquals(newSteerOutput, currentSteerOutput)) { // Save some CAN buffer
            currentSteerOutput = newSteerOutput;
            steerMotor.Set(currentSteerOutput);
            //PIDController.SetReference(steerAngleSetpoint, rev::spark::SparkMax::ControlType::kPosition, rev::spark::ClosedLoopSlot::kSlot0); //added
        }

        if (driveMode == POSITION) {
            driveMotor.set(TalonFXMotor::POSITION, drivePositionSetpoint);
        } else {
            driveMotor.set(TalonFXMotor::VELOCITY, driveVelocitySetpoint);
        }
    }
}

Rotation2d SwerveModule::getSteerEncoder() {
    return steerEnc.getAbsolutePosition();
}

double SwerveModule::getSteerOutput() {
    return currentSteerOutput;
}

double SwerveModule::getDriveEncoderVel() {
    return driveMotor.getVelocity();
}

double SwerveModule::getDriveEncoderPos() { // in rotations
    return driveMotor.getPosition();
}

// TESTING
double SwerveModule::driveMotorDistance(units::second_t timestamp) { // in feet
    if (frc::Timer::GetFPGATimestamp() == timestamp) {
        combinedRot = 0.0;
        driveMotorRot.clear();
        driveMotorRot.push_back(getDriveEncoderPos());
    }
    if (driveMotorRot.size() < 1) {
        driveMotorRot.push_back(getDriveEncoderPos());
    }
    if (driveMotorRot.size() >= 2) {
        if (!(abs(driveMotorRot[0]) > abs(driveMotorRot[1]))) {
            combinedRot += abs(driveMotorRot[1]) - abs(driveMotorRot[0]);
            driveMotorRot.erase(driveMotorRot.begin());
        } else {
            driveMotorRot.erase(driveMotorRot.begin());
        }
    }
    driveMotorRot.push_back(getDriveEncoderPos());
    
    return (combinedRot*wheelCircumFeet)/4.89;
}

/**
 * Set moduleInhibit to true
 * Stops motors and exits PID loop
 * Intended for disabledInit()
 */
void SwerveModule::stopModule() {
    moduleInhibit = true;
}

/**
 * Set moduleInhibit to false
 * Enter PID loop, motors are ON
 * Intended for teleop/auto init functions
 */
void SwerveModule::startModule() {
    moduleInhibit = false;
}
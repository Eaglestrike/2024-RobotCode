#include "Intake/Wrist.h"
#include <iostream>

//constructor j sets motor to brake mode
Wrist::Wrist(bool enabled, bool dbg):
    Mechanism{"Wrist", enabled, dbg},
    m_shuff{"Wrist", dbg}
{
    UpdatePose();
    m_setPt = m_curPos;
    m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Wrist::Zero() {
    m_absEncoderInit = GetAbsEncoderPos();
    m_wristMotor.SetPosition(units::turn_t(0.0));
}


// absolute encoder pos in radians
double Wrist::GetAbsEncoderPos() {
    return m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + WRIST_ABS_ENCODER_OFFSET;
}


// needs to be called INSTEAD of teleop periodic
void Wrist::ManualPeriodic(double wristVolts){
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -MAX_VOLTS, MAX_VOLTS)));
}

void Wrist::CorePeriodic(){
    UpdatePose();
}

// teleop periodic runs on state machine
void Wrist::CoreTeleopPeriodic(){
    double wristVolts = 0;
    switch (m_state){
        case MOVING:
            UpdateTargetPose(); // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                m_state = AT_TARGET;
                ResetPID();
                m_targetPos = m_setPt;
                m_targetVel = 0.0;
                m_targetAcc = 0.0;
            } 
            break;
        case AT_TARGET:
            [[fallthrough]];
        case COAST:
            wristVolts = FFPIDCalculate();
            break;
        case CONST_VOLTAGE:
            wristVolts = m_voltReq;
            break;
        #if WRIST_AUTOTUNING
        case AUTOTUNING:
            m_autoTuner.setPose({m_curPos, m_curVel, m_curAcc});
            wristVolts = m_autoTuner.getVoltage();
        #endif
        default:
            wristVolts = 0.0;
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -MAX_VOLTS, MAX_VOLTS)));
}

void Wrist::MoveTo(double newpos){
    if (newpos == m_setPt) return;
    ChangeSetPt(newpos);
    MoveToSetPt();
}

void Wrist::ChangeSetPt(double newSetpt){
    m_newSetPt = std::clamp(newSetpt, MIN_POS, MAX_POS);
}

void Wrist::Coast(){
    m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_state = COAST;
}

void Wrist::MoveToSetPt(){
    m_setPt = m_newSetPt;
    m_targetPos = m_curPos;
    m_targetVel = 0.0;
    m_targetAcc = MAX_ACC;
    if (m_setPt < m_curPos) m_targetAcc *= -1;
    ResetPID();
    CalcSpeedDecreasePos();
    m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_state = MOVING;
}

//disables
void Wrist::Kill(){
    m_state = STOPPED;
}

Wrist::MechState Wrist::GetState(){
    return m_state;
}

// need to check
double Wrist::GetRelPos() {
    // return m_wristMotor.GetPosition().GetValueAsDouble() * 2 * M_PI;
    return m_wristMotor.GetPosition().GetValueAsDouble()  * 2 * M_PI + m_absEncoderInit;
}

//Updates the current position, velocity, and acceleration of the wrist
void Wrist::UpdatePose(){
    double newPos = GetRelPos(); // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

//updates the trapezoidal motion profile
void Wrist::UpdateTargetPose(){
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel * 0.02;

    if (m_speedDecreasePos < m_setPt){ // if trapezoid is pos
        if (newP > m_speedDecreasePos) // if after turn pt
            newV = std::max(0.0, m_targetVel - MAX_ACC * 0.02);
        else 
            newV = std::min(MAX_VEL, m_targetVel + MAX_ACC * 0.02);
    } else {
        if (newP > m_speedDecreasePos) // if before the turn pt
            newV = std::max(-MAX_VEL, m_targetVel - MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, m_targetVel + MAX_ACC * 0.02);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = MAX_ACC;
    else newA = -MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

//calculates voltage output with feedforwardPID
double Wrist::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    if (fabs(posErr) <= POS_TOLERANCE) posErr =0;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double s = m_s;
    if (m_targetVel < 0) s = -m_s;
    else if (m_targetVel == 0) s = 0;
    double ff = m_g * cos(m_targetPos) + s + m_v*m_targetVel + m_a*m_targetAcc;
    if (shuffleboard_){
        //see how each term is contributing to ff (row 4)
        m_shuff.PutNumber("posErr", posErr); 
        m_shuff.PutNumber("velErr", velErr); 
        m_shuff.PutNumber("ff out", ff); 
        m_shuff.PutNumber("pid out", pid); 
    }
    return pid+ff;
}

void Wrist::CoreShuffleboardInit(){
    //Pose data (row 0)
    m_shuff.add("Pos", &m_curPos, {1,1,0,0},false);
    m_shuff.add("Vel", &m_curVel, {1,1,1,0},false);
    m_shuff.add("Acc", &m_curAcc, {1,1,2,0},false);

    //Test Voltage (row 1)
    m_shuff.add("Voltage", &m_voltReq, {1,1,0,1}, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {1,1,1,1});

    //Feedforward tuning (row 2)
    m_shuff.add("g", &m_g, {1,1,0,2},true);
    m_shuff.add("s", &m_s, {1,1,1,2},true);
    m_shuff.add("v", &m_v, {1,1,2,2},true);
    m_shuff.add("a", &m_a, {1,1,3,2},true);

    //PID tuning (row 3)
    m_shuff.add("p", &m_kp, {1,1,0,3},true);
    m_shuff.add("i", &m_ki, {1,1,1,3},true);
    m_shuff.add("d", &m_kd, {1,1,2,3},true);

    //Deploy (row 3 rightside)
    m_shuff.add("SetPoint", &m_newSetPt, {1,1,4,3}, true);
    m_shuff.addButton("Deploy", [&]{MoveToSetPt();}, {1,1,5,3});
    m_shuff.addButton("Coast", [&]{Coast();}, {1,1,6,3});
    #if WRIST_AUTOTUNING
    m_shuff.addButton("Auto Tune", [&]{m_state = AUTOTUNING;}, {1,1,6,3});
    #endif

    //Debug values (row 4)
    m_shuff.PutNumber("posErr", 0.0, {1,1,0,4}); 
    m_shuff.PutNumber("velErr", 0.0, {1,1,1,4}); 
    m_shuff.PutNumber("ff out", 0.0, {1,1,2,4}); 
    m_shuff.PutNumber("pid out", 0.0, {1,1,3,4}); 
}


void Wrist::CoreShuffleboardPeriodic(){
    m_shuff.update(true);
    #if WRIST_AUTOTUNING
    m_autoTuner.ShuffleboardUpdate();
    #endif
}

//calculates the position at which the speed will begin decreasing
void Wrist::CalcSpeedDecreasePos(){
    double MAX_VEL = MAX_VEL, MAX_ACC = MAX_ACC;
    if(fabs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_speedDecreasePos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_speedDecreasePos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

//returns whether the wrist is at its setpoint
bool Wrist::AtSetpoint(){
    if (fabs(m_curPos - m_setPt) <= POS_TOLERANCE)
        return true;
    return false;
}

void Wrist::ResetPID(){
    m_totalErr = 0;
}

// for tuning, can test constant voltage on wrist or rollers 
// but need to pick which wrist or rollers in the code, since it cant be changed from shuffleboard
void Wrist::SetVoltage(){
    m_state = CONST_VOLTAGE;
    m_voltReq = std::clamp(m_voltReq, -MAX_VOLTS, MAX_VOLTS);
}
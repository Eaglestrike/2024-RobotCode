#include "Intake/Wrist.h"
#include <iostream>

//constructor j sets motor to brake mode
Wrist::Wrist(bool enabled, bool dbg):
    Mechanism{"Wrist", enabled, dbg},
    m_shuff{"Wrist", dbg},
    m_trapezoidalProfile{MAX_VEL, MAX_ACC}
{
    UpdatePose();  
    m_setPt = m_curPos;
    m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Wrist::Zero() {
    m_wristMotor.SetPosition(units::turn_t(0));
}

double Wrist::GetPos(){
    return m_curPos;
}

// needs to be called INSTEAD of teleop periodic
void Wrist::ManualPeriodic(double wristVolts){
    m_voltReq = wristVolts;
    SetVoltage();
}

void Wrist::CorePeriodic(){
    UpdatePose();
}

// teleop periodic runs on state machine
void Wrist::CoreTeleopPeriodic(){
    wristVolts = 0;
    switch (m_state){
        case MOVING:
            // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                m_state = AT_TARGET;
                ResetPID();
            } 
            break;
        case AT_TARGET:
            wristVolts = FFPIDCalculate();
            break;
        case COAST:
            wristVolts = 0.0;
            break;
        case CONST_VOLTAGE:
            wristVolts = m_voltReq;
            break;
        #if WRIST_AUTOTUNING
        case AUTOTUNING:
            m_autoTuner.setPose({m_curPos, m_curVel, m_curAcc});
            wristVolts = m_autoTuner.getVoltage();
            break;
        #endif
        default:
            wristVolts = 0.0;
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -MAX_VOLTS, MAX_VOLTS)));
}


void Wrist::Log(FRCLogger& logger){
    logger.LogNum("wrist volts", wristVolts);
    logger.LogNum("targ pos",  m_trapezoidalProfile.currentPose().pos);
    logger.LogNum("targ vel",  m_trapezoidalProfile.currentPose().vel);
    logger.LogNum("targ acc",  m_trapezoidalProfile.currentPose().acc);
    logger.LogNum("wrist setpt", m_setPt);
    logger.LogNum("pos",  m_curPos);
    logger.LogNum("vel",  m_curVel);
    logger.LogNum("acc",  m_curAcc);
    
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
    if (!m_wasCoasting) {
        // m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        m_wasCoasting = true;
    }
    m_state = COAST;
}

void Wrist::MoveToSetPt(){
    m_setPt = m_newSetPt;
    double vel = std::clamp(m_curVel, -MAX_VEL, MAX_VEL);
    double acc = std::clamp(m_curAcc, -MAX_ACC, MAX_ACC);
    
    m_trapezoidalProfile.setTarget({m_curPos, vel, acc}, {m_setPt, 0.0, 0.0});
    ResetPID();
    m_wasCoasting = false;
    // m_wristMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
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
    return -m_wristMotor.GetPosition().GetValueAsDouble()  * 2 * M_PI * REL_CONV_FACTOR + ENCODER_OFFSET;
}

//Updates the current position, velocity, and acceleration of the wrist
void Wrist::UpdatePose(){
    double newPos = GetRelPos(); // might need to negate or do some wrap around calculations
    double newVel = -m_wristMotor.GetVelocity().GetValueAsDouble() * 2 * M_PI * REL_CONV_FACTOR;
    m_curAcc = -m_wristMotor.GetAcceleration().GetValueAsDouble() * 2 * M_PI * REL_CONV_FACTOR;
    m_curVel = newVel;
    m_curPos = newPos;
}


//calculates voltage output with feedforwardPID
double Wrist::FFPIDCalculate(){
    auto  t= m_trapezoidalProfile.currentPose();
    double targetPos = t.pos, 
    targetVel = t.vel, 
    targetAcc = t.acc;
    double posErr = targetPos - m_curPos, 
    velErr = targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    if (fabs(posErr) <= POS_TOLERANCE) posErr =0;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double s = m_s;
    if (targetVel < 0) s = -m_s;
    else if (targetVel == 0) s = 0;
    double ff = m_g * cos(m_curPos) + s + m_v*targetVel + m_a*targetAcc;
    if (shuffleboard_){
        //see how each term is contributing to ff (row 4)
        m_shuff.PutNumber("pos targ", targetPos); 
        m_shuff.PutNumber("vel targ", targetVel); 

        // m_shuff.PutNumber("posErr", posErr); 
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

    // m_shuff.add("targ Pos", &m_targetPos, {1,1,0,0},false);
    // m_shuff.add("targ Vel", &m_targetVel, {1,1,1,0},false);
    // m_shuff.add("targ Acc", &m_targetAcc, {1,1,2,0},false);
    m_shuff.PutString("State", "", {1,1,3,0});

    //Test Voltage (row 1)
    m_shuff.add("Voltage", &m_voltReq, {1,1,0,1}, true);
    m_shuff.addButton("Set Voltage", [&]{SetVoltage();}, {1,1,1,1});
    m_shuff.add("Max Voltage", &MAX_VOLTS, {1,1,2,1}, true);

    //Feedforward tuning (row 2)
    m_shuff.add("g", &m_g, {1,1,0,2},true);
    m_shuff.add("s", &m_s, {1,1,1,2},true);
    m_shuff.add("v", &m_v, {1,1,2,2},true);
    m_shuff.add("a", &m_a, {1,1,3,2},true);
    m_shuff.add("max v", &MAX_VEL, {1,1,4,2},true);
    m_shuff.add("max a", &MAX_ACC, {1,1,5,2},true);

    //PID tuning (row 3)
    m_shuff.add("p", &m_kp, {1,1,0,3},true);
    m_shuff.add("i", &m_ki, {1,1,1,3},true);
    m_shuff.add("d", &m_kd, {1,1,2,3},true);

    //Deploy (row 3 rightside)
    m_shuff.add("SetPoint", &m_newSetPt, {1,1,4,3}, true);
    m_shuff.addButton("Deploy", [&]{MoveToSetPt();}, {1,1,5,3});
    m_shuff.addButton("Coast", [&]{Coast();}, {1,1,6,3});
    m_shuff.addButton("Zero", [&]{Zero();}, {1,1,7,3});
    #if WRIST_AUTOTUNING
    m_shuff.addButton("Auto Tune", [&]{m_state = AUTOTUNING;}, {1,1,7,3});
    #endif

    //Constants (row 4)
    m_shuff.add("Offset", &ENCODER_OFFSET, {1,1,0,4}, true);
    m_shuff.add("Max Pos", &MAX_POS, {1,1,1,4}, true);
    m_shuff.add("Min Pos", &MIN_POS, {1,1,2,4}, true);
    m_shuff.add("Tolerance", &POS_TOLERANCE, {1,1,3,4}, true);

    //Debug values (row 4 rightside)
    m_shuff.PutNumber("posErr", 0.0, {1,1,5,4}); 
    m_shuff.PutNumber("velErr", 0.0, {1,1,6,4}); 
    m_shuff.PutNumber("ff out", 0.0, {1,1,7,4}); 
    m_shuff.PutNumber("pid out", 0.0, {1,1,8,4});

    m_trapezoidalProfile.setMaxAcc(MAX_ACC);
    m_trapezoidalProfile.setMaxVel(MAX_VEL);
}

// double Wrist::getSetPt(){
//     return m_setPt;
// }

void Wrist::CoreShuffleboardPeriodic(){
    switch(m_state){
        case MOVING:
            m_shuff.PutString("State", "Moving");
            break;
        case AT_TARGET:
            m_shuff.PutString("State", "At Target");
            break;
        case STOPPED:
            m_shuff.PutString("State", "Stopped");
            break;
        case COAST:
            m_shuff.PutString("State", "Coast");
            break;
        case CONST_VOLTAGE:
            m_shuff.PutString("State", "Voltage");
            break;
        default:
            m_shuff.PutString("State", "Other");
    }
    m_shuff.update(true);
    #if WRIST_AUTOTUNING
    m_autoTuner.ShuffleboardUpdate();
    #endif
}

bool Wrist::ProfileDone(){
    return m_trapezoidalProfile.isFinished();
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

/**
 * Gets max volts for wrist
*/
double Wrist::GetMaxVolts() {
    return MAX_VOLTS;
}

double Wrist::GetManualVolts() {
    return MANUAL_MAX_VOLTS;
}
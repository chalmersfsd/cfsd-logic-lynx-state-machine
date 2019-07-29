// These functions are used to print the enum values as strings of their defined names

#ifndef ENUMPRINT_HPP
#define ENUMPRINT_HPP

std::ostream& operator<<(std::ostream& out, const asState value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(asState::AS_OFF);     
        PROCESS_VAL(asState::AS_READY);     
        PROCESS_VAL(asState::AS_DRIVING);
        PROCESS_VAL(asState::AS_FINISHED);
        PROCESS_VAL(asState::AS_EMERGENCY);
        PROCESS_VAL(asState::AS_MANUAL);
    }
#undef PROCESS_VAL

    return out << s;
}

std::ostream& operator<<(std::ostream& out, const ebsState value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(ebsState::EBS_UNAVAILABLE);     
        PROCESS_VAL(ebsState::EBS_ARMED);     
        PROCESS_VAL(ebsState::EBS_ACTIVATED);
    }
#undef PROCESS_VAL

    return out << s;
}

std::ostream& operator<<(std::ostream& out, const ebsInitState value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(ebsInitState::EBS_INIT_ENTRY);     
        PROCESS_VAL(ebsInitState::EBS_INIT_CHARGING);     
        PROCESS_VAL(ebsInitState::EBS_INIT_COMPRESSOR);
        PROCESS_VAL(ebsInitState::EBS_INIT_INITIALIZED);
        PROCESS_VAL(ebsInitState::EBS_INIT_FAILED);
    }
#undef PROCESS_VAL

    return out << s;
}

std::ostream& operator<<(std::ostream& out, const serviceBrakeState value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(serviceBrakeState::BRAKE_UNAVAILABLE);
        PROCESS_VAL(serviceBrakeState::BRAKE_AVAILABLE);     
        PROCESS_VAL(serviceBrakeState::BRAKE_ENGAGED);
    }
#undef PROCESS_VAL

    return out << s;
}

std::ostream& operator<<(std::ostream& out, const asMission value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(asMission::AMI_ACCELERATION);
        PROCESS_VAL(asMission::AMI_AUTOCROSS);     
        PROCESS_VAL(asMission::AMI_BRAKETEST);     
        PROCESS_VAL(asMission::AMI_INSPECTION);     
        PROCESS_VAL(asMission::AMI_MANUAL);     
        PROCESS_VAL(asMission::AMI_NONE);     
        PROCESS_VAL(asMission::AMI_SKIDPAD);
        PROCESS_VAL(asMission::AMI_TRACKDRIVE);
        PROCESS_VAL(asMission::AMI_TEST);
    }
#undef PROCESS_VAL

    return out << s;
}

#endif
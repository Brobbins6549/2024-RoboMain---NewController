#pragma once

#include "commonauto/AutoStep.h"
#include "Intake.h"

class StopIntake : public AutoStep {
    public:
        StopIntake() : AutoStep("StopIntake"){
        }
        void Init() {}
        
        bool Execute() {

            Intake::GetInstance().Speed1(0);
            return true;
        }

    private:
};
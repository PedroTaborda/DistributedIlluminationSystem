#include "controller.hpp"

#include "globals.hpp"
#include "interface.hpp"
#include "utilities.hpp"

Controller::Controller()
{

}

void Controller::setup(float proportionalGain, float integralGain, repeating_timer *timerStruct) volatile{
    this->proportionalGain = proportionalGain;
    this->integralGain = integralGain;

    integralError = 0.f;
    samplingTime = 0.01;
    antiWindup = true;
    feedback = true;
    feedforward = true;
    controllerOn = true;
    simulatorOn = true;
    reference[0] = 0; reference[1] = 0;
    dutyCycle[0] = 0; dutyCycle[1] = 0;
    occupancy = 0;

    simulator.initialize(micros(), measureVoltage(10), dutyCycle[occupancy]);

    int64_t delayUs = (int64_t) (samplingTime * 1e6);

    alarm_pool_add_repeating_timer_us(core1AlarmPool, -delayUs, Controller::controllerLoop, (void *)this, timerStruct);
}

bool Controller::controllerLoop(repeating_timer *timerStruct) {
    volatile Controller* controller = (volatile Controller*)timerStruct->user_data;
    float reference = controller->reference[controller->occupancy];

    // Measure the current luminance value and get the current simulator prediction
    float voltage = measureVoltage(5);
    float currentLux = LDRVoltageToLux(voltage);
    float feedbackReference = 0.f;
    controller->simulatorValue = controller->simulator.getLuminosity(micros());
    if(controller->simulatorOn)
        feedbackReference = controller->simulatorValue;
    else
        feedbackReference = reference;

    if(controller->controllerOn) {
        float feedforwardTerm = 0.f;
        float feedbackTerm = 0.f;
        float proportionalTerm = 0.f;
        float integralTerm = 0.f;
        float dutyCycle = 0.f;

        if(controller->feedforward)
            feedforwardTerm = (reference - ambientIlluminance) / gain;

        if(controller->feedback) {
            // Compute error
            controller->trackingError = feedbackReference - currentLux;

            // Compute proportional and integral terms
            proportionalTerm = controller->proportionalGain * controller->trackingError;

            controller->integralError += controller->samplingTime * controller->trackingError;
            integralTerm = controller->integralGain * controller->integralError;

            feedbackTerm = proportionalTerm + integralTerm;
            
            dutyCycle = feedbackTerm + feedforwardTerm;

            // Implement Anti-Windup by saturating the integral term
            if(controller->antiWindup && !fequal(controller->integralGain, 0.f)) {
                float dutyMin = 0.0f;
                float dutyMax = 1.0f;

                if(dutyCycle < dutyMin) {
                    integralTerm = dutyMin - feedforwardTerm - proportionalTerm;
                    controller->integralError = integralTerm / controller->integralGain;
                } 
                
                if(dutyCycle > dutyMax) {
                    integralTerm = dutyMax - feedforwardTerm - proportionalTerm;
                    controller->integralError = integralTerm / controller->integralGain;
                }
                feedbackTerm = proportionalTerm + integralTerm;
            }
        }

        dutyCycle = feedforwardTerm + feedbackTerm;

        if(dutyCycle < 0.f) dutyCycle = 0.f;
        if(dutyCycle > 1.f) dutyCycle = 1.f;
        controller->dutyCycle[controller->occupancy] = dutyCycle;
    }

    analogWrite(LED_PIN, (int)(controller->dutyCycle[controller->occupancy] * DAC_RANGE));
    controller->jitter = (micros() - controller->lastTimestamp) - ((int)(controller->samplingTime * 1e6));
    controller->lastTimestamp = micros();

    energy += controller->dutyCycle[controller->occupancy] * controller->samplingTime;
    visibilityAccumulator += max(0, reference - currentLux);
    double currentFlicker = currentLux - previousLux;

    if(currentFlicker * previousFlicker < 0)
        flickerAccumulator += (abs(currentFlicker) + abs(previousFlicker)) / (2 * controller->samplingTime);

    previousLux = currentLux;
    previousFlicker = currentFlicker;

    sampleNumber += 1;

    dutyBuffer.insert(controller->dutyCycle[controller->occupancy]);
    luminanceBuffer.insert(currentLux);

    streamer.setNewValues();

    return true;
}

void Controller::changeSimulatorReference(float reference) volatile{
    float dutyCycle = (reference - ambientIlluminance) / gain;

    if(dutyCycle > 1.0f) dutyCycle = 1.0f;
    if(dutyCycle < 0.0f) dutyCycle = 0.0f;

    simulator.changeInput(micros(), dutyCycle, measureVoltage(10));
}

float Controller::getReference() volatile {
    return reference[occupancy];
}

int Controller::getOccupancy() volatile {
    return occupancy;
}

void Controller::setReference(float reference) volatile {
    controllerOn = true;
    this->reference[occupancy] = reference;
    changeSimulatorReference(reference);
}

void Controller::setDutyCycle(float duty) volatile {
    controllerOn = false;
    this->dutyCycle[occupancy] = duty;
}

void Controller::setOccupancy(int occupancy) volatile {
    this->occupancy = occupancy;
}

void Controller::turnControllerOff() volatile {
    controllerOn = false;
}

void Controller::turnControllerOn() volatile {
    controllerOn = true;
}

void Controller::toggleAntiWindup() volatile {
    antiWindup = !antiWindup;
}

void Controller::toggleFeedback() volatile {
    feedback = !feedback;
}

void Controller::toggleFeedforward() volatile {
    feedforward = !feedforward;
}

void Controller::setSimulator(int simulator) volatile {
    simulatorOn = simulator == 0 ? false : true;
}

void Controller::setProportionalGain(float proportionalGain) volatile{
    this->proportionalGain = proportionalGain;
}

void Controller::setIntegralGain(float integralGain) volatile{
    this->integralGain = integralGain;
}

bool Controller::getAntiWindup() volatile{
    return antiWindup;
}

bool Controller::getFeedback() volatile{
    return feedback;
}

bool Controller::getFeedforward() volatile{
    return feedforward;
}

float Controller::getProportionalGain() volatile{
    return proportionalGain;
}

float Controller::getIntegralGain() volatile{
    return integralGain;
}
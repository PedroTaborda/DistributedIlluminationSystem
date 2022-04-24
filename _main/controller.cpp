#include "controller.hpp"

#include "globals.hpp"
#include "utilities.hpp"

volatile Controller controller;
repeating_timer timerStruct;

Controller::Controller() {}

void Controller::setup(float proportionalGain, float integralGain) volatile {
    this->proportionalGain = proportionalGain;
    this->integralGain = integralGain;

    integralError = 0.f;
    samplingTime = 0.01;
    antiWindup = true;
    feedback = true;
    feedforward = true;
    controllerOn = true;
    simulatorOn = true;
    reference[0] = 0;
    reference[1] = 0;
    dutyCycle = 0;
    occupancy = 0;

    latest_sample.reset();

    simulator.initialize(micros(), measureVoltage(10), dutyCycle);

    int64_t delayUs = (int64_t)(samplingTime * 1e6);

    alarm_pool_add_repeating_timer_us(core1AlarmPool, -delayUs, Controller::controllerLoop, (void *)this, &timerStruct);
}

bool Controller::controllerLoop(repeating_timer *timerStruct) {
    /* must be static because repeating timer callbacks can only have one argument */
    volatile Controller *controller = (volatile Controller *)timerStruct->user_data;
    float reference = controller->reference[controller->occupancy];
    unsigned long t;
    controller->handle_requests();

    t = micros();
    controller->sampleDuration = (t - controller->lastTimestamp);
    controller->lastTimestamp = t;

    // Measure the current luminance value and get the current simulator prediction
    float voltage = measureVoltage(5);
    controller->currentLux = LDRVoltageToLux(voltage);
    float feedbackReference = 0.f;
    controller->simulatorValue = controller->simulator.getLuminosity(t);
    if (controller->simulatorOn)
        feedbackReference = controller->simulatorValue;
    else
        feedbackReference = reference;

    if (controller->controllerOn) {
        float feedforwardTerm = 0.f;
        float feedbackTerm = 0.f;
        float proportionalTerm = 0.f;
        float integralTerm = 0.f;
        float dutyCycle = 0.f;

        if (controller->feedforward) feedforwardTerm = (reference - ambientIlluminance) / gain;

        if (controller->feedback) {
            // Compute error
            controller->trackingError = feedbackReference - controller->currentLux;

            // Compute proportional and integral terms
            proportionalTerm = controller->proportionalGain * controller->trackingError;

            controller->integralError += controller->samplingTime * controller->trackingError;
            integralTerm = controller->integralGain * controller->integralError;

            feedbackTerm = proportionalTerm + integralTerm;

            dutyCycle = feedbackTerm + feedforwardTerm;

            // Implement Anti-Windup by saturating the integral term
            if (controller->antiWindup && !fequal(controller->integralGain, 0.f)) {
                float dutyMin = 0.0f;
                float dutyMax = 1.0f;

                if (dutyCycle < dutyMin) {
                    integralTerm = dutyMin - feedforwardTerm - proportionalTerm;
                    controller->integralError = integralTerm / controller->integralGain;
                }

                if (dutyCycle > dutyMax) {
                    integralTerm = dutyMax - feedforwardTerm - proportionalTerm;
                    controller->integralError = integralTerm / controller->integralGain;
                }
                feedbackTerm = proportionalTerm + integralTerm;
            }
        }

        dutyCycle = feedforwardTerm + feedbackTerm;

        if (dutyCycle < 0.f) dutyCycle = 0.f;
        if (dutyCycle > 1.f) dutyCycle = 1.f;
        controller->dutyCycle = dutyCycle;

        set_u(controller->dutyCycle);
    }

    energy += controller->dutyCycle * controller->samplingTime;
    visibilityAccumulator += max(0, reference - controller->currentLux);
    double currentFlicker = controller->currentLux - previousLux;

    if (currentFlicker * previousFlicker < 0)
        flickerAccumulator += (abs(currentFlicker) + abs(previousFlicker)) / (2 * controller->samplingTime);

    previousLux = controller->currentLux;
    previousFlicker = currentFlicker;

    sampleNumber += 1;

    dutyBuffer.insert(controller->dutyCycle);
    luminanceBuffer.insert(controller->currentLux);

    controller->update_outputs();
    return true;
}

void Controller::handle_requests() volatile {
    /* Handle requests from messenger core */
    controllerOn = control_on_req;
    occupancy = occupancy_req;
    if (new_ref) {
        reference[occupancy] = refL_mlux / 1000.0;
        changeSimulatorReference(reference[occupancy]);

        // only let messenger core continue after all is written
        new_ref = false;
    }
}

void Controller::update_outputs() volatile {
    volatile sample_t newsample;
    newsample.dur = sampleDuration;
    newsample.L = currentLux;
    newsample.IntegralError = integralError;
    newsample.TrackingError = trackingError;
    newsample.SimulatorValue = simulatorValue;
    newsample.Reference = reference[occupancy];
    newsample.num = sampleNumber;

    latest_sample.insert(newsample);
}

void Controller::changeSimulatorReference(float reference) volatile {
    float dutyCycle = (reference - ambientIlluminance) / gain;

    if (dutyCycle > 1.0f) dutyCycle = 1.0f;
    if (dutyCycle < 0.0f) dutyCycle = 0.0f;

    simulator.changeInput(micros(), dutyCycle, measureVoltage(10));
}

// Interface functions to be accessed by messenger core

int Controller::getSampleNumber() volatile {
    if (!latest_sample.available())
        return -1;
    else
        return latest_sample.getBegin(1).num;
}

volatile sample_t& Controller::getSample() volatile { return latest_sample.getBegin(1); }

float Controller::getReference() volatile { return reference[occupancy]; }

int Controller::getOccupancy() volatile { return occupancy; }

void Controller::setReference(float reference) volatile {
    /* Set illuminance reference (lux) and turn control on */
    control_on_req = true;
    refL_mlux = (int)(reference * 1000 + 0.5);  // +0.5 to round
    new_ref = true;
    while (new_ref)
        ;  // wait for controller core to acknowledge new refL
}

/*void Controller::setDutyCycle(float duty) volatile {
    turnControllerOff();
    dutyCycle[occupancy] = duty;
    set_u(duty);
}*/

void Controller::setOccupancy(int occupancy_in) volatile {
    occupancy_req = occupancy_in;
    while (occupancy != occupancy_in)
        ;  // wait for occupancy to change
}

void Controller::turnControllerOff() volatile {
    control_on_req = false;
    while (controllerOn)
        ;  // wait for controller core to turn off controller
}

void Controller::turnControllerOn() volatile { control_on_req = true; }

void Controller::setAntiWindup(int val) volatile { antiWindup = val; }

void Controller::setFeedback(int val) volatile { feedback = val; }

void Controller::setFeedforward(int val) volatile { feedforward = val; }

void Controller::setSimulator(int simulator) volatile { simulatorOn = simulator == 0 ? false : true; }

void Controller::setProportionalGain(float proportionalGain_in) volatile { proportionalGain = proportionalGain_in; }

void Controller::setIntegralGain(float integralGain_in) volatile { integralGain = integralGain_in; }

bool Controller::getAntiWindup() volatile { return antiWindup; }

bool Controller::getFeedback() volatile { return feedback; }

bool Controller::getFeedforward() volatile { return feedforward; }

float Controller::getProportionalGain() volatile { return proportionalGain; }

float Controller::getIntegralGain() volatile { return integralGain; }
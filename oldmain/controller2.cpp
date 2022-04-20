#include "controller2.hpp"

Controller::Controller() {}

void Controller::setup(float proportionalGain, float integralGain, repeating_timer *timerStruct) volatile {
    this->proportionalGain = proportionalGain;
    this->integralGain = integralGain;

    integralError = 0.f;
    samplingTime = 0.01;
    antiWindup = true;
    feedback = true;
    feedforward = true;
    controllerOn = true;
    control_on_req = true;
    simulatorOn = true;
    reference[0] = 0;
    reference[1] = 0;
    refL_mlux = 0;
    new_ref = false;
    dutyCycle = 0;
    occupancy = 0;
    occupancy_req = 0;

    lastTimestamp = micros();
    energy = 0;
    visibilityAccumulator = 0;
    flickerAccumulator = 0;
    previousFlicker = 0;
    previousLux = 0;

    latest_sample.reset();
    luminanceBuffer.reset();
    dutyBuffer.reset();
    sampleNumber = 0;

    simulator.initialize(micros(), measureVoltage(10), dutyCycle);

    int64_t delayUs = (int64_t)(samplingTime * 1e6);

    alarm_pool_add_repeating_timer_us(core1AlarmPool, -delayUs, Controller::controllerLoop, (void *)this, timerStruct);
}

bool Controller::controllerLoop(repeating_timer *timerStruct) {
    /* must be static because repeating timer callbacks can only have one argument */
    volatile Controller *controller = (volatile Controller *)timerStruct->user_data;
    unsigned long t;

    controller->handle_requests();

    // just to avoid typing everytime
    float reference = controller->reference[controller->occupancy];

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

    controller->update_outputs();
    return true;
}

void Controller::handle_requests() volatile {
    /* Handle requests from messenger core */
    controllerOn = control_on_req;
    proportionalGain = proportionalGain_req;
    integralGain = integralGain_req;
    occupancy = occupancy_req;
    if (new_ref) {
        reference[occupancy] = refL_mlux / 1000.0;
        changeSimulatorReference(reference[occupancy]);

        // only let messenger core continue after all is written
        new_ref = false;
    }
}

void Controller::update_outputs() volatile {
    sample_t newsample;
    sampleNumber += 1;

    newsample.dur = sampleDuration;
    newsample.L = currentLux;
    newsample.IntegralError = integralError;
    newsample.TrackingError = trackingError;
    newsample.SimulatorValue = simulatorValue;
    newsample.Reference = reference[occupancy];
    newsample.num = sampleNumber;

    latest_sample.insert(newsample);

    energy += dutyCycle * samplingTime;
    energy_out = (float)energy;

    visibilityAccumulator += max(0, reference[occupancy] - currentLux);
    visibilityAccumulator_out = (float)visibilityAccumulator;

    double currentFlicker = currentLux - previousLux;

    if (currentFlicker * previousFlicker < 0) {
        flickerAccumulator += (abs(currentFlicker) + abs(previousFlicker)) / (2 * samplingTime);
        flickerAccumulator_out = (float)flickerAccumulator;
    }

    previousLux = currentLux;
    previousFlicker = currentFlicker;

    dutyBuffer.insert(dutyCycle);
    luminanceBuffer.insert(currentLux);
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
        return 0;
    else
        return latest_sample.getBegin(1).num;
}

sample_t Controller::getSample() volatile {
    sample_t latest;
    // The latest_sample is a buffer with two entries so that if controller is writing to one entry,
    // the other is read uncorrupted. However, if the method is interrupted in between reading the
    // current head and reading the sample for long enough, the controller might start the next
    // iteration loop, after which it will write to the same entry this function is going to read.
    // To prevent this, turn off interrupts so there is no big delay between reading current head
    // and the sample.
    noInterrupts();
    latest = latest_sample.getBegin(1);
    interrupts();
    return latest;
}

float Controller::getReference() volatile { return reference[occupancy]; }

int Controller::getOccupancy() volatile { return occupancy; }

float Controller::getDutyCycle() volatile { return dutyCycle; }

float Controller::getIlluminance() volatile { return currentLux; }

float Controller::getEnergySpent() volatile { return energy_out; }

float Controller::getVisibilityAccumulator() volatile { return visibilityAccumulator_out; }

float Controller::getFlickerAccumulator() volatile { return flickerAccumulator_out; }

void Controller::getDutyBuffer(float out[60 * 100]) volatile {
    // index from current head in case controller increments the buffer during the read
    int currentHead = dutyBuffer.getCurrentHead();
    for (int i = 0; i < 60 * 100; i++) {
        out[i] = dutyBuffer.indexFromCustomHead(i - (60 * 100 - 1), currentHead);
    }
}

void Controller::getIlluminanceBuffer(float out[60 * 100]) volatile {
    // index from current head in case controller increments the buffer during the read
    int currentHead = luminanceBuffer.getCurrentHead();
    for (int i = 0; i < 60 * 100; i++) {
        out[i] = luminanceBuffer.indexFromCustomHead(i - (60 * 100 - 1), currentHead);
    }
}

void Controller::setReference(float reference) volatile {
    /* Set illuminance reference (lux) and turn control on */
    control_on_req = true;
    refL_mlux = (int)(reference * 1000 + 0.5);  // +0.5 to round
    new_ref = true;
    while (new_ref)
        ;  // wait for controller core to acknowledge new refL
}

void Controller::setDutyCycle(float duty) volatile {
    turnControllerOff();
    dutyCycle = duty;
    set_u(duty);
}

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

void Controller::toggleAntiWindup() volatile { antiWindup = !antiWindup; }

void Controller::toggleFeedback() volatile { feedback = !feedback; }

void Controller::toggleFeedforward() volatile { feedforward = !feedforward; }

void Controller::setSimulator(int simulator) volatile { simulatorOn = simulator == 0 ? false : true; }

void Controller::setProportionalGain(float proportionalGain_in) volatile { proportionalGain_req = proportionalGain_in; }

void Controller::setIntegralGain(float integralGain_in) volatile { integralGain_req = integralGain_in; }

bool Controller::getAntiWindup() volatile { return antiWindup; }

bool Controller::getFeedback() volatile { return feedback; }

bool Controller::getFeedforward() volatile { return feedforward; }

float Controller::getProportionalGain() volatile { return proportionalGain; }

float Controller::getIntegralGain() volatile { return integralGain; }
// MIT License
//
// Copyright 2024 Eurofurence e.V. 
// 
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

/**
 * @author Honigeintopf
 */

#include <Arduino.h>
#include <Preferences.h>

#include <EFLed.h>
#include <EFLogging.h>

#include "FSM.h"

Preferences pref;

FSM::FSM(unsigned int tickrate_ms)
: state(nullptr)
, tickrate_ms(tickrate_ms)
, state_last_run(0)
{
    this->globals = std::make_shared<FSMGlobals>();
    this->state = std::make_unique<DisplayPrideFlag>();
    this->state->attachGlobals(this->globals);
}

FSM::~FSM() {
    this->state->exit();
}

void FSM::resume() {
    // Restore FSM data
    this->restoreGlobals();

    // Restore LED brightness setting
    EFLed.setBrightnessPercent(this->globals->ledBrightnessPercent);
    
    // Resume last remembered state
    switch (this->globals->resumeStateIdx) {
        case 0: this->transition(std::make_unique<DisplayPrideFlag>()); break;
        case 1: this->transition(std::make_unique<AnimateRainbow>()); break;
        case 2: this->transition(std::make_unique<AnimateMatrix>()); break;
        case 3: this->transition(std::make_unique<AnimateSnake>()); break;
        case 4: this->transition(std::make_unique<AnimateHeartbeat>()); break;
		case 6: this->transition(std::make_unique<GameHuemesh>()); break;
		case 7: this->transition(std::make_unique<VUMeter>()); break;
        default:
            LOGF_WARNING("(FSM) Failed to resume to unknown state: %d\r\n", this->globals->resumeStateIdx);
            this->transition(std::make_unique<DisplayPrideFlag>());
            break;

    }
}

void FSM::transition(std::unique_ptr<FSMState> next) {
    if (next == nullptr) {
        LOG_WARNING("(FSM) Failed to transition to null state. Aborting.");
        return;
    }

    // State exit
    LOGF_INFO("(FSM) Transition %s -> %s\r\n", this->state->getName(), next->getName());
    this->state->exit();

    // Persist globals if state dirtied it or next state wants to be persisted
    if (next->shouldBeRemembered()) {
        this->globals->resumeStateIdx = this->globals->menuMainPointerIdx;
    }
    if (this->state->isGlobalsDirty() || next->shouldBeRemembered()) {
        this->persistGlobals();
        this->state->resetGlobalsDirty();
    }

    // Transition to next state
    this->state = std::move(next);
    this->state->attachGlobals(this->globals);
    this->state_last_run = 0;
    this->state->entry();
}

unsigned int FSM::getTickRateMs() {
    return this->tickrate_ms;
}

void FSM::queueEvent(FSMEvent event) {
    FSMEvent test = std::move(event);
    noInterrupts();
    {
        this->eventqueue.push(test);
    }
    interrupts();
}

unsigned int FSM::getQueueSize() {
    unsigned int queuesize;

    noInterrupts();
    {
        queuesize = this->eventqueue.size();
    }
    interrupts();

    return queuesize;
}

FSMEvent FSM::dequeueEvent() {
    FSMEvent event = FSMEvent::NoOp;

    noInterrupts();
    {
        if (this->eventqueue.size() > 0) {
            event = std::move(this->eventqueue.front());
            this->eventqueue.pop();
        }
    }
    interrupts();

    return event;
}

void FSM::handle() {
    this->handle(999);
}

void FSM::handle(unsigned int num_events) {
    // Handle dirtied FSM globals
    if (this->state->isGlobalsDirty()) {
        this->persistGlobals();
        this->state->resetGlobalsDirty();
    }

    // Handle state run()
    if (
        this->state->getTickRateMs() == 0 ||
        millis() >= this->state_last_run + this->state->getTickRateMs()
    ) {
        this->state_last_run = millis();
        this->state->run();
    }

    // Handle events
    for (; num_events > 0; num_events--) {
        FSMEvent event = this->dequeueEvent();
        std::unique_ptr<FSMState> next = nullptr;

        // Propagate event to current state
        switch(event) {
            case FSMEvent::FingerprintTouch:
                next = this->state->touchEventFingerprintTouch();
                break;
            case FSMEvent::FingerprintRelease:
                next = this->state->touchEventFingerprintRelease();
                break;
            case FSMEvent::FingerprintShortpress:
                next = this->state->touchEventFingerprintShortpress();
                break;
            case FSMEvent::FingerprintLongpress:
                next = this->state->touchEventFingerprintLongpress();
                break;
            case FSMEvent::NoseTouch:
                next = this->state->touchEventNoseTouch();
                break;
            case FSMEvent::NoseRelease:
                next = this->state->touchEventNoseRelease();
                break;
            case FSMEvent::NoseShortpress:
                next = this->state->touchEventNoseShortpress();
                break;
            case FSMEvent::NoseLongpress:
                next = this->state->touchEventNoseLongpress();
                break;
            case FSMEvent::AllShortpress:
                next = this->state->touchEventAllShortpress();
                break;
            case FSMEvent::AllLongpress:
                next = this->state->touchEventAllLongpress();
                break;
            case FSMEvent::NoOp:
                return;
            default:
                LOGF_WARNING("(FSM) Failed to handle unknown event: %d\r\n", event);
                return;
        } 

        // Handle state transition
        if (next != nullptr) {
            this->transition(move(next));
        }
    }
}

void FSM::persistGlobals() {
    pref.begin(this->NVS_NAMESPACE, false);
    pref.clear();
    pref.putUInt("resumeStateIdx", this->globals->resumeStateIdx);
    pref.putUInt("menuIdx", this->globals->menuMainPointerIdx);
    pref.putUInt("prideFlagMode", this->globals->prideFlagModeIdx);
    pref.putUInt("animRainbow", this->globals->animRainbowIdx);
    pref.putUInt("animSnakeIdx", this->globals->animSnakeAnimationIdx);
    pref.putUInt("animSnakeHueIdx", this->globals->animSnakeHueIdx);
    pref.putUInt("animHbHue", this->globals->animHeartbeatHue);
    pref.putUInt("animHbSpeed", this->globals->animHeartbeatSpeed);
    pref.putUInt("animMatrixIdx", this->globals->animMatrixIdx);
    pref.putUInt("ledBrightPcent", this->globals->ledBrightnessPercent);
	pref.putUInt("huemeshOwnHue", this->globals->huemeshOwnHue);
    pref.end();
}

void FSM::restoreGlobals() {
    pref.begin(this->NVS_NAMESPACE, true);
    this->globals->resumeStateIdx = pref.getUInt("resumeStateIdx", random(0, 3));
    this->globals->menuMainPointerIdx = pref.getUInt("menuIdx", 0);
    this->globals->prideFlagModeIdx = pref.getUInt("prideFlagMode", 1);
    this->globals->animRainbowIdx = pref.getUInt("animRainbow", 0);
    this->globals->animSnakeAnimationIdx = pref.getUInt("animSnakeIdx", 0);
    this->globals->animSnakeHueIdx = pref.getUInt("animSnakeHueIdx", 0);
    this->globals->animHeartbeatHue = pref.getUInt("animHbHue", 0);
    this->globals->animHeartbeatSpeed = pref.getUInt("animHbSpeed", 1);
    this->globals->animMatrixIdx = pref.getUInt("animMatrixIdx", 0);
    this->globals->ledBrightnessPercent = pref.getUInt("ledBrightPcent", 40);
	this->globals->huemeshOwnHue = pref.getUInt("huemeshOwnHue", 0);
    pref.end();
}

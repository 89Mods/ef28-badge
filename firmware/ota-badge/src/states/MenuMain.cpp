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

#include <EFLed.h>

#include "FSMState.h"

const char* MenuMain::getName() {
    return "MenuMain";
}

void MenuMain::entry() {
    EFLed.clear();
    EFLed.setDragonEye(CRGB::Green);


    menucursor_idx = 0;
    EFLed.setEFBarCursor(menucursor_idx, CRGB::Purple, CRGB::Black);
}

void MenuMain::exit() {
    EFLed.clear();
}

std::unique_ptr<FSMState> MenuMain::touchEventFingerprintTouch() {
    return nullptr;
}

std::unique_ptr<FSMState> MenuMain::touchEventFingerprintRelease() {
    EFLed.setEFBarCursor((++menucursor_idx % 11), CRGB::Purple, CRGB::Black);
    return nullptr;
}

std::unique_ptr<FSMState> MenuMain::touchEventFingerprintShortpress() {
    return nullptr;
}

std::unique_ptr<FSMState> MenuMain::touchEventFingerprintLongpress() {
    return std::make_unique<DisplayPrideFlag>();
}

std::unique_ptr<FSMState> MenuMain::touchEventNoseLongpress() {
    return std::make_unique<MenuOTAUpdate>();
}

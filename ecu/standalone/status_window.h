#ifndef ECU_STANDALONE_STATUS_WINDOW_H
#define ECU_STANDALONE_STATUS_WINDOW_H

#include "status.h"

#include <atomic>

class StimulusLink;

// Status window (Windows): is the ECU running, is it receiving the engine's
// samples, XCP, calibration, datalog, live values and messages. Runs on the
// calling thread until the window is closed (which sets stop) or stop is set
// elsewhere (Ctrl+C). The ECU loop ending by itself leaves the window open,
// showing it stopped. stimulus: inputs editable from the window (or nullptr).
void runStatusWindow(StatusBoard &board, StimulusLink *stimulus, std::atomic<bool> &stop);

#endif /* ECU_STANDALONE_STATUS_WINDOW_H */

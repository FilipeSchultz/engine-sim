#ifndef ATG_ENGINE_SIM_ECU_DCM_FILE_H
#define ATG_ENGINE_SIM_ECU_DCM_FILE_H

#include "calibration.h"

#include <string>
#include <vector>

namespace ecu {
    // ASAM DCM 2.0 (text calibration dataset, as exchanged with CANape / INCA).
    // Supported blocks: FESTWERT (value) and KENNFELD (map); other blocks are
    // skipped with a warning.
    //
    // Reading starts from *page, so parameters missing from the file keep their
    // current values. On failure *page is left untouched.
    bool readDcm(
        const std::string &path,
        CalibrationPage *page,
        std::vector<std::string> *warnings,
        std::string *error);

    bool writeDcm(
        const std::string &path,
        const CalibrationPage &page,
        const std::string &comment,
        std::string *error);
}

#endif /* ATG_ENGINE_SIM_ECU_DCM_FILE_H */

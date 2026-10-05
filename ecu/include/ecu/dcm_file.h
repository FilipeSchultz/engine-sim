#ifndef ECU_DCM_FILE_H
#define ECU_DCM_FILE_H

#include "calibration.h"

#include <string>
#include <vector>

namespace ecu {
    // ASAM DCM 2.0 (text calibration dataset, as exchanged with CANape / INCA).
    // Supported blocks: FESTWERT (value), KENNLINIE (curve) and KENNFELD (map);
    // other blocks are skipped with a warning.
    //
    // Reading starts from *page, so parameters missing from the file keep their
    // current values (reported in warnings). On failure *page is left untouched.
    bool readDcm(
        const std::string &path,
        const CalibrationLayout &layout,
        CalibrationPage *page,
        std::vector<std::string> *warnings,
        std::string *error);

    // Written to a temporary file first and then renamed over the target, so
    // an interrupted save never leaves a half-written dataset
    bool writeDcm(
        const std::string &path,
        const CalibrationLayout &layout,
        const CalibrationPage &page,
        const std::string &comment,
        std::string *error);

    // Copies an existing file to <dir>/backup/<name>_<yyyymmdd_hhmmss><ext>.
    // Nothing to do (and true) if the file doesn't exist.
    bool backupFile(const std::string &path, std::string *backupPath, std::string *error);
}

#endif /* ECU_DCM_FILE_H */

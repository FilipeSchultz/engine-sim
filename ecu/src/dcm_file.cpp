#include "ecu/dcm_file.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace {
    using Type = ecu::ParameterInfo::Type;

    // Splits a line into tokens, keeping "quoted strings" together
    std::vector<std::string> tokenize(const std::string &line) {
        std::vector<std::string> tokens;
        size_t i = 0;
        while (i < line.size()) {
            if (std::isspace(static_cast<unsigned char>(line[i]))) {
                ++i;
            }
            else if (line[i] == '"') {
                const size_t end = line.find('"', i + 1);
                tokens.push_back(line.substr(i + 1, (end == std::string::npos) ? std::string::npos : end - i - 1));
                i = (end == std::string::npos) ? line.size() : end + 1;
            }
            else {
                size_t end = i;
                while (end < line.size() && !std::isspace(static_cast<unsigned char>(line[end]))) ++end;
                tokens.push_back(line.substr(i, end - i));
                i = end;
            }
        }

        return tokens;
    }

    bool parseNumbers(
        const std::vector<std::string> &tokens,
        size_t first,
        std::vector<float> *out)
    {
        for (size_t i = first; i < tokens.size(); ++i) {
            char *end = nullptr;
            const float v = std::strtof(tokens[i].c_str(), &end);
            if (end == tokens[i].c_str() || *end != '\0') return false;
            out->push_back(v);
        }

        return true;
    }

    std::string at(int line, const std::string &message) {
        return "line " + std::to_string(line) + ": " + message;
    }

    const char *keywordOf(Type type) {
        switch (type) {
        case Type::Value: return "FESTWERT";
        case Type::Curve: return "KENNLINIE";
        default: return "KENNFELD";
        }
    }

    struct Block {
        std::string keyword;
        std::string name;
        int startLine = 0;
        int sizeX = 1;
        int sizeY = 1;

        std::vector<float> axisX;
        std::vector<float> axisY;
        std::vector<std::vector<float>> rows;   // KENNFELD: one row per ST/Y
        std::vector<float> values;              // FESTWERT, KENNLINIE
    };

    bool applyBlock(
        const Block &block,
        const ecu::CalibrationLayout &layout,
        ecu::CalibrationPage *page,
        std::vector<std::string> *warnings,
        std::string *error)
    {
        const ecu::ParameterInfo *p = layout.find(block.name);
        if (p == nullptr) {
            warnings->push_back(at(block.startLine, "unknown parameter '" + block.name + "' ignored"));
            return true;
        }

        if (block.keyword != keywordOf(p->type)) {
            *error = at(block.startLine, block.name + " is a " + keywordOf(p->type) + ", not a " + block.keyword);
            return false;
        }

        float *values = &(*page)[p->offset];

        if (p->type == Type::Value) {
            if (block.values.size() != 1) {
                *error = at(block.startLine, block.name + ": FESTWERT needs exactly one WERT");
                return false;
            }

            values[0] = block.values[0];
            return true;
        }

        if (block.sizeX != p->sizeX || block.sizeY != p->sizeY) {
            std::stringstream ss;
            ss << block.name << ": size " << block.sizeX;
            if (p->type == Type::Map) ss << "x" << block.sizeY;
            ss << " but the ECU expects " << p->sizeX;
            if (p->type == Type::Map) ss << "x" << p->sizeY;
            *error = at(block.startLine, ss.str());
            return false;
        }

        if ((int)block.axisX.size() != p->sizeX) {
            *error = at(block.startLine, block.name + ": ST/X needs " + std::to_string(p->sizeX) + " values");
            return false;
        }

        float *axisX = &(*page)[p->offsetX];
        for (int i = 0; i < p->sizeX; ++i) axisX[i] = block.axisX[i];

        if (p->type == Type::Curve) {
            if ((int)block.values.size() != p->sizeX) {
                *error = at(block.startLine, block.name + ": WERT needs " + std::to_string(p->sizeX) + " values");
                return false;
            }

            for (int i = 0; i < p->sizeX; ++i) values[i] = block.values[i];
            return true;
        }

        if ((int)block.axisY.size() != p->sizeY || (int)block.rows.size() != p->sizeY) {
            *error = at(block.startLine, block.name + ": needs " + std::to_string(p->sizeY) + " ST/Y rows");
            return false;
        }

        float *axisY = &(*page)[p->offsetY];
        for (int j = 0; j < p->sizeY; ++j) {
            if ((int)block.rows[j].size() != p->sizeX) {
                *error = at(block.startLine,
                    block.name + ": row " + std::to_string(j + 1) + " needs " + std::to_string(p->sizeX) + " values");
                return false;
            }

            axisY[j] = block.axisY[j];
            for (int i = 0; i < p->sizeX; ++i) {
                values[j * p->sizeX + i] = block.rows[j][i];
            }
        }

        return true;
    }
}

bool ecu::readDcm(
    const std::string &path,
    const CalibrationLayout &layout,
    CalibrationPage *page,
    std::vector<std::string> *warnings,
    std::string *error)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        *error = "cannot open " + path;
        return false;
    }

    CalibrationPage result = *page;
    std::set<std::string> found;
    Block block;
    bool inBlock = false;
    bool skipping = false;

    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;

        const std::vector<std::string> tokens = tokenize(line);
        if (tokens.empty() || tokens[0][0] == '*') continue;

        const std::string &keyword = tokens[0];

        if (!inBlock) {
            if (keyword == "KONSERVIERUNG_FORMAT") continue;

            if (keyword == "FESTWERT" || keyword == "KENNLINIE" || keyword == "KENNFELD") {
                const size_t expectedTokens =
                    (keyword == "FESTWERT") ? 2 : (keyword == "KENNLINIE") ? 3 : 4;
                if (tokens.size() < expectedTokens) {
                    *error = at(lineNumber, keyword + " header incomplete");
                    return false;
                }

                block = Block();
                block.keyword = keyword;
                block.name = tokens[1];
                block.startLine = lineNumber;
                if (keyword != "FESTWERT") block.sizeX = std::atoi(tokens[2].c_str());
                if (keyword == "KENNFELD") block.sizeY = std::atoi(tokens[3].c_str());

                inBlock = true;
                skipping = false;
            }
            else if (keyword == "FUNKTIONEN" || keyword == "FESTWERTEBLOCK"
                || keyword == "GRUPPENKENNFELD" || keyword == "GRUPPENKENNLINIE" || keyword == "STUETZSTELLENVERTEILUNG"
                || keyword == "TEXTSTRING" || keyword == "FESTKENNFELD" || keyword == "FESTKENNLINIE")
            {
                if (keyword != "FUNKTIONEN") {
                    warnings->push_back(at(lineNumber, keyword + " blocks are not supported, skipped"));
                }

                inBlock = true;
                skipping = true;
            }
            else {
                *error = at(lineNumber, "unexpected '" + keyword + "'");
                return false;
            }

            continue;
        }

        if (keyword == "END") {
            if (!skipping) {
                if (!applyBlock(block, layout, &result, warnings, error)) return false;
                found.insert(block.name);
            }
            inBlock = false;
            continue;
        }

        if (skipping) continue;

        if (keyword == "LANGNAME" || keyword == "FUNKTION" || keyword == "DISPLAYNAME"
            || keyword == "EINHEIT_X" || keyword == "EINHEIT_Y" || keyword == "EINHEIT_W"
            || keyword == "VAR")
        {
            continue;
        }

        std::vector<float> numbers;
        if (keyword == "WERT") {
            if (!parseNumbers(tokens, 1, &numbers)) {
                *error = at(lineNumber, "invalid number in WERT");
                return false;
            }

            if (block.keyword == "KENNFELD") {
                if (block.rows.empty()) {
                    *error = at(lineNumber, "WERT before ST/Y");
                    return false;
                }
                block.rows.back().insert(block.rows.back().end(), numbers.begin(), numbers.end());
            }
            else {
                block.values.insert(block.values.end(), numbers.begin(), numbers.end());
            }
        }
        else if (keyword == "ST/X") {
            if (!parseNumbers(tokens, 1, &numbers)) {
                *error = at(lineNumber, "invalid number in ST/X");
                return false;
            }
            block.axisX.insert(block.axisX.end(), numbers.begin(), numbers.end());
        }
        else if (keyword == "ST/Y") {
            if (!parseNumbers(tokens, 1, &numbers) || numbers.size() != 1) {
                *error = at(lineNumber, "ST/Y needs exactly one value");
                return false;
            }
            block.axisY.push_back(numbers[0]);
            block.rows.emplace_back();
        }
        else {
            *error = at(lineNumber, "unexpected '" + keyword + "' inside " + block.name);
            return false;
        }
    }

    if (inBlock) {
        *error = at(lineNumber, "missing END");
        return false;
    }

    if (!layout.validate(result, error)) return false;

    int missing = 0;
    std::string missingNames;
    for (const ParameterInfo &p : layout.parameters()) {
        if (found.count(p.name) == 0) {
            if (missing++ < 3) missingNames += (missingNames.empty() ? "" : ", ") + p.name;
        }
    }
    if (missing > 0) {
        warnings->push_back(std::to_string(missing) + " parameter(s) not in the file kept their values ("
            + missingNames + (missing > 3 ? ", ..." : "") + ")");
    }

    *page = result;
    return true;
}

bool ecu::writeDcm(
    const std::string &path,
    const CalibrationLayout &layout,
    const CalibrationPage &page,
    const std::string &comment,
    std::string *error)
{
    const std::string temporaryPath = path + ".tmp";
    std::ofstream file(temporaryPath);
    if (!file.is_open()) {
        *error = "cannot write " + temporaryPath;
        return false;
    }

    char buffer[64];
    auto number = [&buffer](float v, const char *format) {
        std::snprintf(buffer, sizeof(buffer), format, v);
        return std::string(buffer);
    };

    file << "* engine-sim ECU calibration dataset\n";
    if (!comment.empty()) {
        std::stringstream ss(comment);
        std::string commentLine;
        while (std::getline(ss, commentLine)) file << "* " << commentLine << "\n";
    }
    file << "\nKONSERVIERUNG_FORMAT 2.0\n";

    std::string function;
    for (const ParameterInfo &p : layout.parameters()) {
        if (p.function != function) {
            function = p.function;
            file << "\n* --- " << function << "\n";
        }

        const float *values = &page[p.offset];
        file << "\n";

        if (p.type == Type::Value) {
            file << "FESTWERT " << p.name << "\n";
            file << "   LANGNAME \"" << p.description << "\"\n";
            file << "   EINHEIT_W \"" << p.unit << "\"\n";
            file << "   WERT " << number(values[0], "%g") << "\n";
            file << "END\n";
            continue;
        }

        const float *axisX = &page[p.offsetX];

        if (p.type == Type::Curve) {
            file << "KENNLINIE " << p.name << " " << p.sizeX << "\n";
            file << "   LANGNAME \"" << p.description << "\"\n";
            file << "   EINHEIT_X \"" << p.unitX << "\"\n";
            file << "   EINHEIT_W \"" << p.unit << "\"\n";
            file << "   ST/X";
            for (int i = 0; i < p.sizeX; ++i) file << number(axisX[i], " %8g");
            file << "\n   WERT";
            for (int i = 0; i < p.sizeX; ++i) file << number(values[i], " %8.3f");
            file << "\nEND\n";
            continue;
        }

        const float *axisY = &page[p.offsetY];

        file << "KENNFELD " << p.name << " " << p.sizeX << " " << p.sizeY << "\n";
        file << "   LANGNAME \"" << p.description << "\"\n";
        file << "   EINHEIT_X \"" << p.unitX << "\"\n";
        file << "   EINHEIT_Y \"" << p.unitY << "\"\n";
        file << "   EINHEIT_W \"" << p.unit << "\"\n";

        file << "   ST/X";
        for (int i = 0; i < p.sizeX; ++i) file << number(axisX[i], " %8g");
        file << "\n";

        for (int j = 0; j < p.sizeY; ++j) {
            file << "   ST/Y " << number(axisY[j], "%g") << "\n";
            file << "   WERT";
            for (int i = 0; i < p.sizeX; ++i) {
                file << number(values[j * p.sizeX + i], " %8.3f");
            }
            file << "\n";
        }

        file << "END\n";
    }

    file.close();
    if (!file) {
        *error = "cannot write " + temporaryPath;
        return false;
    }

    std::error_code ec;
    std::filesystem::rename(temporaryPath, path, ec);
    if (ec) {
        *error = "cannot replace " + path + ": " + ec.message();
        return false;
    }

    return true;
}

bool ecu::backupFile(const std::string &path, std::string *backupPath, std::string *error) {
    namespace fs = std::filesystem;

    std::error_code ec;
    const fs::path source(path);
    if (!fs::exists(source, ec)) return true;

    const std::time_t now = std::time(nullptr);
    std::tm local = {};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);

    const fs::path directory = source.parent_path() / "backup";
    fs::create_directories(directory, ec);
    const fs::path target =
        directory / (source.stem().string() + "_" + stamp + source.extension().string());

    fs::copy_file(source, target, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        *error = "cannot back up " + path + ": " + ec.message();
        return false;
    }

    if (backupPath != nullptr) *backupPath = target.string();
    return true;
}

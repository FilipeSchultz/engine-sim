#include "../../include/ecu/dcm_file.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {
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

    struct Block {
        std::string keyword;
        std::string name;
        int startLine = 0;
        int sizeX = 0;
        int sizeY = 0;

        std::vector<float> axisX;
        std::vector<float> axisY;
        std::vector<std::vector<float>> rows;
        std::vector<float> values;
    };

    bool applyBlock(
        const Block &block,
        ecu::CalibrationPage *page,
        std::vector<std::string> *warnings,
        std::string *error)
    {
        const ecu::ParameterInfo *p = ecu::findParameter(block.name);
        if (p == nullptr) {
            warnings->push_back(at(block.startLine, "unknown parameter '" + block.name + "' ignored"));
            return true;
        }

        if (block.keyword == "FESTWERT") {
            if (p->type != ecu::ParameterInfo::Type::Value) {
                *error = at(block.startLine, block.name + " is a map, not a FESTWERT");
                return false;
            }
            if (block.values.size() != 1) {
                *error = at(block.startLine, block.name + ": FESTWERT needs exactly one WERT");
                return false;
            }

            ecu::parameterValues(page, *p)[0] = block.values[0];
            return true;
        }

        // KENNFELD
        if (p->type != ecu::ParameterInfo::Type::Map) {
            *error = at(block.startLine, block.name + " is a value, not a KENNFELD");
            return false;
        }

        if (block.sizeX != p->sizeX || block.sizeY != p->sizeY) {
            std::stringstream ss;
            ss << block.name << ": size " << block.sizeX << "x" << block.sizeY
                << " but the ECU expects " << p->sizeX << "x" << p->sizeY;
            *error = at(block.startLine, ss.str());
            return false;
        }

        if ((int)block.axisX.size() != p->sizeX) {
            *error = at(block.startLine, block.name + ": ST/X needs " + std::to_string(p->sizeX) + " values");
            return false;
        }

        if ((int)block.axisY.size() != p->sizeY || (int)block.rows.size() != p->sizeY) {
            *error = at(block.startLine, block.name + ": needs " + std::to_string(p->sizeY) + " ST/Y rows");
            return false;
        }

        for (int j = 0; j < p->sizeY; ++j) {
            if ((int)block.rows[j].size() != p->sizeX) {
                *error = at(block.startLine,
                    block.name + ": row " + std::to_string(j + 1) + " needs " + std::to_string(p->sizeX) + " values");
                return false;
            }
        }

        float *axisX = ecu::parameterAxis(page, p->offsetX);
        float *axisY = ecu::parameterAxis(page, p->offsetY);
        float *values = ecu::parameterValues(page, *p);
        for (int i = 0; i < p->sizeX; ++i) axisX[i] = block.axisX[i];
        for (int j = 0; j < p->sizeY; ++j) {
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

            if (keyword == "FESTWERT" || keyword == "KENNFELD") {
                const size_t expectedTokens = (keyword == "FESTWERT") ? 2 : 4;
                if (tokens.size() < expectedTokens) {
                    *error = at(lineNumber, keyword + " header incomplete");
                    return false;
                }

                block = Block();
                block.keyword = keyword;
                block.name = tokens[1];
                block.startLine = lineNumber;
                if (keyword == "KENNFELD") {
                    block.sizeX = std::atoi(tokens[2].c_str());
                    block.sizeY = std::atoi(tokens[3].c_str());
                }

                inBlock = true;
                skipping = false;
            }
            else if (keyword == "FUNKTIONEN" || keyword == "KENNLINIE" || keyword == "FESTWERTEBLOCK"
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
            if (!skipping && !applyBlock(block, &result, warnings, error)) return false;
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

            if (block.keyword == "FESTWERT") {
                block.values.insert(block.values.end(), numbers.begin(), numbers.end());
            }
            else {
                if (block.rows.empty()) {
                    *error = at(lineNumber, "WERT before ST/Y");
                    return false;
                }
                block.rows.back().insert(block.rows.back().end(), numbers.begin(), numbers.end());
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

    if (!validate(result, error)) return false;

    *page = result;
    return true;
}

bool ecu::writeDcm(
    const std::string &path,
    const CalibrationPage &page,
    const std::string &comment,
    std::string *error)
{
    std::ofstream file(path);
    if (!file.is_open()) {
        *error = "cannot write " + path;
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

    for (const ParameterInfo &p : getParameters()) {
        const float *values = parameterValues(&page, p);
        file << "\n";

        if (p.type == ParameterInfo::Type::Value) {
            file << "FESTWERT " << p.name << "\n";
            file << "   LANGNAME \"" << p.description << "\"\n";
            file << "   EINHEIT_W \"" << p.unit << "\"\n";
            file << "   WERT " << number(values[0], "%g") << "\n";
            file << "END\n";
            continue;
        }

        const float *axisX = parameterAxis(&page, p.offsetX);
        const float *axisY = parameterAxis(&page, p.offsetY);

        file << "KENNFELD " << p.name << " " << p.sizeX << " " << p.sizeY << "\n";
        file << "   LANGNAME \"" << p.description << "\"\n";
        file << "   EINHEIT_X \"" << p.unitX << "\"\n";
        file << "   EINHEIT_Y \"" << p.unitY << "\"\n";
        file << "   EINHEIT_W \"" << p.unit << "\"\n";

        file << "   ST/X";
        for (int i = 0; i < p.sizeX; ++i) file << number(axisX[i], " %8.0f");
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

    return true;
}

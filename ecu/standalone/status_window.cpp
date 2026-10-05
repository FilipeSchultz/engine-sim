#include "status_window.h"

#include "io_link.h"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <cwchar>
#include <string>

namespace {
    constexpr UINT_PTR RefreshTimer = 1;
    constexpr UINT RefreshPeriod_ms = 200;

    constexpr int StimulusEditId = 101;
    constexpr int MessagesId = 102;

    // Without a sample for longer than this the link is reported lost (the
    // I/O controller sends one every 10 ms)
    constexpr double SampleTimeout_s = 0.5;

    // Layout at 96 dpi
    constexpr int Margin = 16;
    constexpr int ClientWidth = 620;
    constexpr int ClientHeight = 760;
    constexpr int RowsTop = 92;
    constexpr int RowHeight = 48;
    constexpr int Rows = 4;
    constexpr int ValuesTop = RowsTop + Rows * RowHeight + 12;
    constexpr int ValueRows = 6;
    constexpr int ValueRowHeight = 22;
    constexpr int StimulusTop = ValuesTop + 28 + ValueRows * ValueRowHeight + 10;
    constexpr int MessagesTop = StimulusTop + 44;

    const COLORREF Background = RGB(250, 250, 250);
    const COLORREF Text = RGB(33, 33, 33);
    const COLORREF Muted = RGB(110, 110, 110);
    const COLORREF Separator = RGB(224, 224, 224);
    const COLORREF Green = RGB(46, 125, 50);
    const COLORREF Amber = RGB(230, 145, 0);
    const COLORREF Red = RGB(198, 40, 40);
    const COLORREF Gray = RGB(158, 158, 158);
    const COLORREF Blue = RGB(21, 101, 192);

    struct Indicator {
        COLORREF color;
        std::wstring state;
        std::wstring detail;
    };

    std::wstring widen(const std::string &text) {
        if (text.empty()) return {};
        const int length = MultiByteToWideChar(CP_ACP, 0, text.data(), (int)text.size(), nullptr, 0);
        std::wstring wide(length, L'\0');
        MultiByteToWideChar(CP_ACP, 0, text.data(), (int)text.size(), &wide[0], length);
        return wide;
    }

    std::string narrow(const std::wstring &text) {
        if (text.empty()) return {};
        const int length = WideCharToMultiByte(CP_ACP, 0, text.data(), (int)text.size(), nullptr, 0, nullptr, nullptr);
        std::string result(length, '\0');
        WideCharToMultiByte(CP_ACP, 0, text.data(), (int)text.size(), &result[0], length, nullptr, nullptr);
        return result;
    }

    std::wstring format(const wchar_t *pattern, ...) {
        wchar_t buffer[512];
        va_list args;
        va_start(args, pattern);
        std::vswprintf(buffer, sizeof(buffer) / sizeof(buffer[0]), pattern, args);
        va_end(args);
        return buffer;
    }

    double secondsSince(EcuStatus::Clock::time_point t) {
        return std::chrono::duration<double>(EcuStatus::Clock::now() - t).count();
    }

    const wchar_t *onOff(bool on) { return on ? L"ON" : L"OFF"; }

    class StatusWindow {
        public:
            StatusWindow(StatusBoard &board, StimulusLink *stimulus, std::atomic<bool> &stop)
                : m_board(board), m_stimulus(stimulus), m_stop(stop) { /* void */ }

            ~StatusWindow() {
                for (HFONT font : { m_titleFont, m_boldFont, m_normalFont, m_smallFont, m_valueFont }) {
                    if (font != nullptr) DeleteObject(font);
                }
            }

            void run();

        private:
            static LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
            LRESULT handle(UINT message, WPARAM wParam, LPARAM lParam);

            int px(int v) const { return MulDiv(v, m_dpi, 96); }
            HFONT createFont(int points, int weight, const wchar_t *face) const;

            void createControls();
            void layout();
            void refresh();
            void applyStimulus();

            void paint(HDC dc);
            void drawText(HDC dc, int x, int y, int width, const std::wstring &text, HFONT font, COLORREF color, UINT align = DT_LEFT);
            void drawDot(HDC dc, int x, int y, int diameter, COLORREF color);
            void drawSeparator(HDC dc, int y);
            void drawIndicator(HDC dc, int row, const wchar_t *label, const Indicator &indicator);

            Indicator linkIndicator() const;
            Indicator xcpIndicator() const;
            Indicator calibrationIndicator() const;
            Indicator datalogIndicator() const;

            StatusBoard &m_board;
            StimulusLink *m_stimulus;
            std::atomic<bool> &m_stop;
            EcuStatus m_status;

            HWND m_hwnd = nullptr;
            HWND m_stimulusEdit = nullptr;
            HWND m_stimulusApply = nullptr;
            HWND m_messages = nullptr;
            int m_messagesExtent = 0;

            int m_dpi = 96;
            int m_width = ClientWidth;          // client width at 96 dpi
            HFONT m_titleFont = nullptr;
            HFONT m_boldFont = nullptr;
            HFONT m_normalFont = nullptr;
            HFONT m_smallFont = nullptr;
            HFONT m_valueFont = nullptr;
    };

    HFONT StatusWindow::createFont(int points, int weight, const wchar_t *face) const {
        return CreateFontW(-MulDiv(points, m_dpi, 72), 0, 0, 0, weight, FALSE, FALSE, FALSE,
            DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
            DEFAULT_PITCH, face);
    }

    void StatusWindow::run() {
        SetProcessDPIAware();
        {
            HDC screen = GetDC(nullptr);
            m_dpi = GetDeviceCaps(screen, LOGPIXELSY);
            ReleaseDC(nullptr, screen);
        }

        m_titleFont = createFont(16, FW_SEMIBOLD, L"Segoe UI");
        m_boldFont = createFont(10, FW_SEMIBOLD, L"Segoe UI");
        m_normalFont = createFont(10, FW_NORMAL, L"Segoe UI");
        m_smallFont = createFont(9, FW_NORMAL, L"Segoe UI");
        m_valueFont = createFont(10, FW_SEMIBOLD, L"Consolas");

        const HINSTANCE instance = GetModuleHandleW(nullptr);
        WNDCLASSEXW windowClass = {};
        windowClass.cbSize = sizeof(windowClass);
        windowClass.lpfnWndProc = windowProc;
        windowClass.hInstance = instance;
        windowClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
        windowClass.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
        windowClass.lpszClassName = L"EcuStandaloneStatus";
        RegisterClassExW(&windowClass);

        const DWORD style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN;
        RECT frame = { 0, 0, px(ClientWidth), px(ClientHeight) };
        AdjustWindowRect(&frame, style, FALSE);

        m_status = m_board.snapshot();
        m_hwnd = CreateWindowExW(WS_EX_CONTROLPARENT, windowClass.lpszClassName, L"ECU",
            style, CW_USEDEFAULT, CW_USEDEFAULT, frame.right - frame.left, frame.bottom - frame.top,
            nullptr, nullptr, instance, this);
        if (m_hwnd == nullptr) return;

        createControls();
        layout();
        refresh();
        ShowWindow(m_hwnd, SW_SHOWNORMAL);
        SetTimer(m_hwnd, RefreshTimer, RefreshPeriod_ms, nullptr);

        MSG message;
        while (GetMessageW(&message, nullptr, 0, 0) > 0) {
            // Tab between the controls, Enter in the stimulus box = Apply
            if (!IsDialogMessageW(m_hwnd, &message)) {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }
        }
    }

    LRESULT CALLBACK StatusWindow::windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        if (message == WM_NCCREATE) {
            auto create = reinterpret_cast<CREATESTRUCTW *>(lParam);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
            reinterpret_cast<StatusWindow *>(create->lpCreateParams)->m_hwnd = hwnd;
        }

        auto window = reinterpret_cast<StatusWindow *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        return (window != nullptr)
            ? window->handle(message, wParam, lParam)
            : DefWindowProcW(hwnd, message, wParam, lParam);
    }

    LRESULT StatusWindow::handle(UINT message, WPARAM wParam, LPARAM lParam) {
        switch (message) {
            case WM_TIMER:
                if (m_stop) {
                    DestroyWindow(m_hwnd);
                }
                else {
                    refresh();
                }
                return 0;

            case WM_COMMAND:
                if (LOWORD(wParam) == IDOK) {
                    applyStimulus();
                    return 0;
                }
                break;

            case WM_SIZE:
                layout();
                InvalidateRect(m_hwnd, nullptr, FALSE);
                return 0;

            case WM_GETMINMAXINFO: {
                auto info = reinterpret_cast<MINMAXINFO *>(lParam);
                RECT frame = { 0, 0, px(560), px(MessagesTop + 120) };
                AdjustWindowRect(&frame, WS_OVERLAPPEDWINDOW, FALSE);
                info->ptMinTrackSize.x = frame.right - frame.left;
                info->ptMinTrackSize.y = frame.bottom - frame.top;
                return 0;
            }

            case WM_ERASEBKGND:
                return 1;

            case WM_PAINT: {
                PAINTSTRUCT ps;
                HDC dc = BeginPaint(m_hwnd, &ps);

                // Drawn off screen, then copied: no flicker on refresh
                RECT client;
                GetClientRect(m_hwnd, &client);
                HDC memory = CreateCompatibleDC(dc);
                HBITMAP bitmap = CreateCompatibleBitmap(dc, client.right, client.bottom);
                HGDIOBJ previous = SelectObject(memory, bitmap);

                HBRUSH background = CreateSolidBrush(Background);
                FillRect(memory, &client, background);
                DeleteObject(background);
                paint(memory);

                BitBlt(dc, 0, 0, client.right, client.bottom, memory, 0, 0, SRCCOPY);
                SelectObject(memory, previous);
                DeleteObject(bitmap);
                DeleteDC(memory);

                EndPaint(m_hwnd, &ps);
                return 0;
            }

            case WM_CLOSE:
                m_stop = true;
                DestroyWindow(m_hwnd);
                return 0;

            case WM_DESTROY:
                KillTimer(m_hwnd, RefreshTimer);
                PostQuitMessage(0);
                return 0;
        }

        return DefWindowProcW(m_hwnd, message, wParam, lParam);
    }

    void StatusWindow::createControls() {
        const HINSTANCE instance = GetModuleHandleW(nullptr);

        if (m_stimulus != nullptr) {
            m_stimulusEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", widen(m_stimulus->values()).c_str(),
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                0, 0, 0, 0, m_hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(StimulusEditId)), instance, nullptr);
            m_stimulusApply = CreateWindowExW(0, L"BUTTON", L"Apply",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
                0, 0, 0, 0, m_hwnd, reinterpret_cast<HMENU>(IDOK), instance, nullptr);
            SendMessageW(m_stimulusEdit, WM_SETFONT, reinterpret_cast<WPARAM>(m_normalFont), TRUE);
            SendMessageW(m_stimulusApply, WM_SETFONT, reinterpret_cast<WPARAM>(m_normalFont), TRUE);
        }

        m_messages = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", nullptr,
            WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_HSCROLL | WS_TABSTOP | LBS_NOINTEGRALHEIGHT,
            0, 0, 0, 0, m_hwnd, reinterpret_cast<HMENU>(static_cast<INT_PTR>(MessagesId)), instance, nullptr);
        SendMessageW(m_messages, WM_SETFONT, reinterpret_cast<WPARAM>(m_smallFont), TRUE);
    }

    void StatusWindow::layout() {
        RECT client;
        GetClientRect(m_hwnd, &client);
        m_width = MulDiv(client.right, 96, m_dpi);

        if (m_stimulusEdit != nullptr) {
            const int buttonWidth = 84;
            const int left = Margin + 76;
            MoveWindow(m_stimulusEdit, px(left), px(StimulusTop), px(m_width - left - Margin - buttonWidth - 8), px(26), TRUE);
            MoveWindow(m_stimulusApply, px(m_width - Margin - buttonWidth), px(StimulusTop), px(buttonWidth), px(26), TRUE);
        }

        const int top = px(MessagesTop + 24);
        MoveWindow(m_messages, px(Margin), top,
            client.right - 2 * px(Margin), std::max<int>(client.bottom - px(Margin) - top, px(40)), TRUE);
    }

    void StatusWindow::refresh() {
        m_status = m_board.snapshot();

        const std::vector<std::string> messages = m_board.takeMessages();
        if (!messages.empty()) {
            HDC dc = GetDC(m_messages);
            HGDIOBJ previous = SelectObject(dc, m_smallFont);
            for (const std::string &text : messages) {
                const std::wstring line = widen(text);
                SendMessageW(m_messages, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(line.c_str()));

                SIZE size;
                GetTextExtentPoint32W(dc, line.c_str(), (int)line.size(), &size);
                m_messagesExtent = std::max(m_messagesExtent, (int)size.cx + px(8));
            }
            SelectObject(dc, previous);
            ReleaseDC(m_messages, dc);

            const LRESULT count = SendMessageW(m_messages, LB_GETCOUNT, 0, 0);
            for (LRESULT i = count; i > 500; --i) {
                SendMessageW(m_messages, LB_DELETESTRING, 0, 0);
            }
            SendMessageW(m_messages, LB_SETHORIZONTALEXTENT, m_messagesExtent, 0);
            SendMessageW(m_messages, LB_SETTOPINDEX, std::max<LRESULT>(SendMessageW(m_messages, LB_GETCOUNT, 0, 0) - 1, 0), 0);
        }

        const std::wstring title = m_status.software.empty()
            ? std::wstring(L"ECU")
            : L"ECU - " + widen(m_status.software);
        SetWindowTextW(m_hwnd, title.c_str());

        InvalidateRect(m_hwnd, nullptr, FALSE);
    }

    void StatusWindow::applyStimulus() {
        if (m_stimulus == nullptr) return;

        const int length = GetWindowTextLengthW(m_stimulusEdit);
        std::wstring text(length + 1, L'\0');
        GetWindowTextW(m_stimulusEdit, &text[0], length + 1);
        text.resize(length);

        std::string error;
        if (m_stimulus->set(narrow(text), &error)) {
            m_board.addMessage("stimulus: " + m_stimulus->values());
            SetWindowTextW(m_stimulusEdit, widen(m_stimulus->values()).c_str());
        }
        else {
            m_board.addMessage("stimulus: " + error);
        }
        refresh();
    }

    void StatusWindow::drawText(HDC dc, int x, int y, int width, const std::wstring &text, HFONT font, COLORREF color, UINT align) {
        SelectObject(dc, font);
        SetTextColor(dc, color);
        SetBkMode(dc, TRANSPARENT);
        RECT rect = { px(x), px(y), px(x + width), px(y + 40) };
        if ((align & DT_PATH_ELLIPSIS) == 0) align |= DT_END_ELLIPSIS;
        DrawTextW(dc, text.c_str(), (int)text.size(), &rect, align | DT_SINGLELINE | DT_NOPREFIX);
    }

    void StatusWindow::drawDot(HDC dc, int x, int y, int diameter, COLORREF color) {
        HBRUSH brush = CreateSolidBrush(color);
        HGDIOBJ previousBrush = SelectObject(dc, brush);
        HGDIOBJ previousPen = SelectObject(dc, GetStockObject(NULL_PEN));
        Ellipse(dc, px(x), px(y), px(x + diameter) + 1, px(y + diameter) + 1);
        SelectObject(dc, previousPen);
        SelectObject(dc, previousBrush);
        DeleteObject(brush);
    }

    void StatusWindow::drawSeparator(HDC dc, int y) {
        RECT rect = { px(Margin), px(y), px(m_width - Margin), px(y) + 1 };
        HBRUSH brush = CreateSolidBrush(Separator);
        FillRect(dc, &rect, brush);
        DeleteObject(brush);
    }

    void StatusWindow::drawIndicator(HDC dc, int row, const wchar_t *label, const Indicator &indicator) {
        const int y = RowsTop + row * RowHeight;
        drawDot(dc, Margin, y + 4, 14, indicator.color);
        drawText(dc, Margin + 24, y, 110, label, m_boldFont, Text);
        drawText(dc, Margin + 134, y, m_width - Margin * 2 - 134, indicator.state, m_boldFont, indicator.color);
        drawText(dc, Margin + 134, y + 21, m_width - Margin * 2 - 134, indicator.detail, m_smallFont, Muted, DT_LEFT | DT_PATH_ELLIPSIS);
    }

    Indicator StatusWindow::linkIndicator() const {
        const EcuStatus &s = m_status;
        const std::wstring link = widen(s.link);

        if (s.state == EcuStatus::State::Starting) return { Gray, L"Starting", link };
        if (s.state == EcuStatus::State::Failed) return { Gray, L"Not started", L"" };
        if (s.linkClosed) return { Red, L"Link closed", link };
        if (s.state == EcuStatus::State::Stopped) return { Gray, L"Stopped", link };
        if (s.samples == 0) return { Amber, L"Waiting for the I/O controller", link };

        const double age = secondsSince(s.lastSample);
        std::wstring detail = link + format(L"  |  %.0f samples/s", s.sampleRate_Hz);
        if (s.lostSamples > 0) detail += format(L", %llu lost", (unsigned long long)s.lostSamples);

        if (age > SampleTimeout_s) {
            return { Red, format(L"No data for %.1f s", age), detail };
        }
        if (!s.linkHasController) {
            return { Blue, L"Stimulus: inputs generated here, no engine", detail };
        }
        return { Green, L"Communicating with the engine", detail };
    }

    Indicator StatusWindow::xcpIndicator() const {
        const EcuStatus &s = m_status;
        const std::wstring endpoint = widen(s.xcpEndpoint);

        if (!s.xcpEnabled) return { Gray, L"Off", L"start with --xcp to calibrate with VISION / CANape" };
        if (s.state == EcuStatus::State::Starting) return { Gray, L"Starting", endpoint };
        if (!s.xcpRunning) {
            return { Red, L"Not running", endpoint.empty() ? L"see the messages" : endpoint + L"  |  see the messages" };
        }
        if (s.xcpConnected) return { Green, L"Tool connected", endpoint };
        return { Amber, L"Waiting for a tool", endpoint };
    }

    Indicator StatusWindow::calibrationIndicator() const {
        const EcuStatus &s = m_status;
        const std::wstring detail = format(L"fuel %ls, ignition %ls  |  ",
            onOff(s.fuelEnabled), onOff(s.ignitionEnabled)) + widen(s.calibrationFile);

        if (s.state == EcuStatus::State::Starting) return { Gray, L"Loading", widen(s.calibrationFile) };
        if (s.state == EcuStatus::State::Failed) return { Gray, L"-", widen(s.calibrationFile) };
        if (s.xcpConnected) return { Blue, L"Owned by the XCP tool (file reload paused)", detail };
        if (!s.calibrationFileOk) return { Red, L"File rejected: previous values kept (see the messages)", detail };
        return { Green, L"Loaded (reloaded when the file is saved)", detail };
    }

    Indicator StatusWindow::datalogIndicator() const {
        if (m_status.logFile.empty()) return { Gray, L"Off", L"start with --log <file.csv>" };
        if (m_status.state != EcuStatus::State::Running) return { Gray, L"Not recording", widen(m_status.logFile) };
        return { Green, L"Recording (20 Hz)", widen(m_status.logFile) };
    }

    void StatusWindow::paint(HDC dc) {
        const EcuStatus &s = m_status;

        // Header: what this is, and whether it runs
        drawText(dc, Margin, 12, 300, L"Virtual ECU", m_titleFont, Text);

        COLORREF stateColor = Gray;
        const wchar_t *stateText = L"STARTING";
        switch (s.state) {
            case EcuStatus::State::Running: stateColor = Green; stateText = L"RUNNING"; break;
            case EcuStatus::State::Stopped: stateColor = Gray; stateText = L"STOPPED"; break;
            case EcuStatus::State::Failed: stateColor = Red; stateText = L"FAILED"; break;
            default: break;
        }

        const int pillWidth = 120;
        const int pillLeft = m_width - Margin - pillWidth;
        HBRUSH brush = CreateSolidBrush(stateColor);
        HGDIOBJ previousBrush = SelectObject(dc, brush);
        HGDIOBJ previousPen = SelectObject(dc, GetStockObject(NULL_PEN));
        RoundRect(dc, px(pillLeft), px(14), px(pillLeft + pillWidth), px(44), px(14), px(14));
        SelectObject(dc, previousPen);
        SelectObject(dc, previousBrush);
        DeleteObject(brush);
        drawText(dc, pillLeft, 19, pillWidth, stateText, m_boldFont, RGB(255, 255, 255), DT_CENTER);

        std::wstring subtitle = s.software.empty() ? L"" : L"Software " + widen(s.software);
        if (s.state == EcuStatus::State::Running) {
            const int up = (int)secondsSince(s.started);
            subtitle += format(L"  |  up %d:%02d:%02d  |  ECU time %.1f s",
                up / 3600, (up / 60) % 60, up % 60, s.signals.time_s);
        }
        drawText(dc, Margin, 50, m_width - 2 * Margin, subtitle, m_smallFont, Muted);
        drawSeparator(dc, RowsTop - 12);

        drawIndicator(dc, 0, L"Engine link", linkIndicator());
        drawIndicator(dc, 1, L"XCP", xcpIndicator());
        drawIndicator(dc, 2, L"Calibration", calibrationIndicator());
        drawIndicator(dc, 3, L"Datalog", datalogIndicator());
        drawSeparator(dc, ValuesTop - 6);

        // Live values: what the ECU sees and what it commands
        const ecu::EcuSignals &v = s.signals;
        const ecu::EcuOutputs &out = s.outputs;
        const bool live = s.state == EcuStatus::State::Running && s.samples > 0;
        auto value = [&](const wchar_t *pattern, double x) {
            return live ? format(pattern, x) : std::wstring(L"-");
        };

        const wchar_t *engineState = L"stopped";
        if (v.engineState == ecu::Cranking) engineState = L"cranking";
        else if (v.engineState == ecu::Running) engineState = L"running";

        const std::wstring left[ValueRows][2] = {
            { L"Engine speed", value(L"%.0f rpm", v.rpm) },
            { L"MAP", value(L"%.1f kPa", v.map_kPa) },
            { L"Throttle", value(L"%.1f %%", v.tps_percent) },
            { L"Lambda", live && v.lambdaSensor > 0 ? format(L"%.3f", v.lambdaSensor) : std::wstring(L"-") },
            { L"Coolant", value(L"%.1f °C", v.clt_C) },
            { L"Intake air", value(L"%.1f °C", v.iat_C) }
        };
        const std::wstring right[ValueRows][2] = {
            { L"Engine state", live ? std::wstring(engineState) : L"-" },
            { L"Injection", !live ? L"-" : out.fuelControl
                ? format(L"%.3f ms @ %.0f°", out.injection_ms, out.injectionAngle_deg)
                : std::wstring(L"off (not ECU)") },
            { L"Spark advance", !live ? L"-" : out.sparkControl
                ? format(L"%.1f° BTDC", out.sparkAdvance_deg)
                : std::wstring(L"off (not ECU)") },
            { L"Lambda corr.", value(L"%.3f", v.fuelCorrLambda) },
            { L"Fuel cut", live ? std::wstring(v.fuelCut >= 0.5 ? L"YES (rev limiter)" : L"no") : L"-" },
            { L"Battery", value(L"%.2f V", v.battery_V) }
        };

        drawText(dc, Margin, ValuesTop, 200, L"Live values", m_boldFont, Text);
        const int column = (m_width - 2 * Margin) / 2;
        for (int i = 0; i < ValueRows; ++i) {
            const int y = ValuesTop + 28 + i * ValueRowHeight;
            drawText(dc, Margin, y, 110, left[i][0], m_normalFont, Muted);
            drawText(dc, Margin + 110, y, column - 120, left[i][1], m_valueFont, Text);
            drawText(dc, Margin + column, y, 110, right[i][0], m_normalFont, Muted);
            drawText(dc, Margin + column + 110, y, column - 110, right[i][1], m_valueFont, Text);
        }

        if (m_stimulusEdit != nullptr) {
            drawText(dc, Margin, StimulusTop + 4, 76, L"Stimulus", m_boldFont, Text);
        }
        drawSeparator(dc, MessagesTop - 2);
        drawText(dc, Margin, MessagesTop + 2, 200, L"Messages", m_boldFont, Text);
    }
}

void runStatusWindow(StatusBoard &board, StimulusLink *stimulus, std::atomic<bool> &stop) {
    StatusWindow window(board, stimulus, stop);
    window.run();
}

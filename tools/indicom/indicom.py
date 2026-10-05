"""IndiCom virtual: combustion analysis for engine-sim's cylinder pressure stream.

Start engine-sim with ENGINE_SIM_INDI_UDP=127.0.0.1:5005, then run:
    .venv\\Scripts\\python indicom.py
"""

import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox, QPushButton,
    QSpinBox, QSplitter, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout,
    QWidget,
)

from acquisition import Acquisition
from analysis import Geometry, Settings, mean_cycle

ENGINES_DIR = Path(__file__).parent / "engines"
DEFAULT_PORT = 5005

RESULT_COLUMNS = [
    ("Pmax [bar]", "pmax_bar", 1),
    ("aPmax [°]", "pmax_angle", 1),
    ("IMEP líq. [bar]", "imep_net_bar", 2),
    ("IMEP bruto [bar]", "imep_gross_bar", 2),
    ("CA10 [°]", "ca10", 1),
    ("CA50 [°]", "ca50", 1),
    ("CA90 [°]", "ca90", 1),
    ("Duração 10-90 [°]", "burn_duration", 1),
    ("COV IMEP [%]", None, 2),
    ("COV Pmax [%]", None, 1),
    ("σ CA50 [°]", None, 1),
    ("Falhas", None, 0),
    ("Ciclos", None, 0),
]

# Display of the pressure / heat release curves
DISPLAY_MEAN = 0            # ensemble average of the last N cycles
DISPLAY_ENVELOPE = 1        # average + min / max band
DISPLAY_OVERLAY = 2         # average + the last cycles in thin lines
DISPLAY_SINGLE = 3          # last cycle only
DISPLAY_MODES = [
    "Ciclo médio",
    "Ciclo médio + envelope mín/máx",
    "Ciclo médio + ciclos sobrepostos",
    "Último ciclo",
]
OVERLAY_CYCLES = 20


def spin(minimum, maximum, value, decimals=1, step=1.0, suffix=""):
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    box.setSuffix(suffix)
    return box


class IndiComWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IndiCom virtual — engine-sim")
        self.resize(1500, 900)

        self.acquisition = None
        self.cylinders = 0
        self.history = {}       # cylinder -> deque of CycleResult
        self.curves = {}        # cylinder -> deque of CycleCurves (display grid)
        self.dirty = set()      # cylinders with new cycles since the last plot update

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_config_panel())
        splitter.addWidget(self._build_main_area())
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        self.status = QLabel()
        self.statusBar().addWidget(self.status, 1)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh)
        self.ui_timer.start(100)

        self._connect()

    # Layout

    def _build_config_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        connection = QGroupBox("Aquisição (UDP)")
        form = QFormLayout(connection)
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(DEFAULT_PORT)
        reconnect = QPushButton("Reconectar")
        reconnect.clicked.connect(self._connect)
        form.addRow("Porta", self.port)
        form.addRow(reconnect)
        self.engine_label = QLabel("—")
        self.engine_label.setWordWrap(True)
        form.addRow("Motor no simulador", self.engine_label)
        layout.addWidget(connection)

        geometry = QGroupBox("Geometria do motor")
        form = QFormLayout(geometry)
        self.bore = spin(10, 500, 100, 2, 0.1, " mm")
        self.stroke = spin(10, 500, 90, 2, 0.1, " mm")
        self.rod = spin(10, 1000, 150, 2, 0.1, " mm")
        self.compression = spin(2, 30, 10, 2, 0.1)
        form.addRow("Diâmetro", self.bore)
        form.addRow("Curso", self.stroke)
        form.addRow("Biela", self.rod)
        form.addRow("Taxa de compressão", self.compression)
        layout.addWidget(geometry)

        tdc = QGroupBox("Cilindros: análise e PMS de combustão (encoder)")
        tdc_layout = QVBoxLayout(tdc)
        self.tdc_table = QTableWidget(0, 2)
        self.tdc_table.setHorizontalHeaderLabels(["Analisar", "PMS [°]"])
        self.tdc_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.tdc_table.itemChanged.connect(self._apply_configuration)
        tdc_layout.addWidget(self.tdc_table)
        selection = QHBoxLayout()
        select_all = QPushButton("Todos")
        select_all.clicked.connect(lambda: self._select_cylinders(True))
        select_none = QPushButton("Nenhum")
        select_none.clicked.connect(lambda: self._select_cylinders(False))
        selection.addWidget(select_all)
        selection.addWidget(select_none)
        tdc_layout.addLayout(selection)
        layout.addWidget(tdc, 1)

        analysis = QGroupBox("Análise")
        form = QFormLayout(analysis)
        self.gamma = spin(1.1, 1.5, 1.33, 3, 0.01)
        self.pegging = QComboBox()
        self.pegging.addItems(["Termodinâmico (politrópico)", "Nenhum (pressão absoluta)"])
        self.polytropic = spin(1.1, 1.5, 1.32, 3, 0.01)
        self.average_cycles = QSpinBox()
        self.average_cycles.setRange(1, 1000)
        self.average_cycles.setValue(50)
        self.display_mode = QComboBox()
        self.display_mode.addItems(DISPLAY_MODES)
        self.display_mode.setCurrentIndex(DISPLAY_ENVELOPE)
        form.addRow("γ (liberação de calor)", self.gamma)
        form.addRow("Pegging", self.pegging)
        form.addRow("Expoente politrópico", self.polytropic)
        form.addRow("Ciclos na média", self.average_cycles)
        form.addRow("Exibição", self.display_mode)
        layout.addWidget(analysis)

        buttons = QHBoxLayout()
        load_sim = QPushButton("Carregar do simulador")
        load_sim.clicked.connect(self._load_from_simulator)
        save = QPushButton("Salvar…")
        save.clicked.connect(self._save_engine)
        open_ = QPushButton("Abrir…")
        open_.clicked.connect(self._open_engine)
        buttons.addWidget(load_sim)
        buttons.addWidget(save)
        buttons.addWidget(open_)
        layout.addLayout(buttons)

        for widget in (self.bore, self.stroke, self.rod, self.compression, self.gamma, self.polytropic):
            widget.valueChanged.connect(self._apply_configuration)
        self.pegging.currentIndexChanged.connect(self._apply_configuration)
        self.average_cycles.valueChanged.connect(self._resize_history)
        self.display_mode.currentIndexChanged.connect(self._redraw)

        panel.setMinimumWidth(330)
        return panel

    def _build_main_area(self):
        area = QSplitter(Qt.Orientation.Vertical)

        pg.setConfigOptions(antialias=True)
        tabs = QTabWidget()

        self.p_theta = pg.PlotWidget()
        self.p_theta.setLabel("bottom", "Ângulo [° depois do PMS]")
        self.p_theta.setLabel("left", "Pressão [bar]")
        self.p_theta.showGrid(x=True, y=True, alpha=0.3)
        self.p_theta.addLegend()
        self.plot_items = {}
        self.p_theta.setXRange(-180, 180)
        tabs.addTab(self.p_theta, "Pressão × ângulo")

        self.p_v = pg.PlotWidget()
        self.p_v.setLogMode(x=True, y=True)
        self.p_v.setLabel("bottom", "Volume [cm³]")
        self.p_v.setLabel("left", "Pressão [bar]")
        self.p_v.showGrid(x=True, y=True, alpha=0.3)
        tabs.addTab(self.p_v, "p × V (log)")

        burn = pg.GraphicsLayoutWidget()
        self.mfb_plot = burn.addPlot(row=0, col=0)
        self.mfb_plot.setLabel("left", "Fração queimada")
        self.mfb_plot.showGrid(x=True, y=True, alpha=0.3)
        self.mfb_plot.setXRange(-60, 120)
        self.heat_plot = burn.addPlot(row=1, col=0)
        self.heat_plot.setLabel("left", "Liberação de calor [J/°]")
        self.heat_plot.setLabel("bottom", "Ângulo [° depois do PMS]")
        self.heat_plot.showGrid(x=True, y=True, alpha=0.3)
        self.heat_plot.setXLink(self.mfb_plot)
        tabs.addTab(burn, "Queima")

        # Cycle-based results, as the trend view of an indicating system
        trend = pg.GraphicsLayoutWidget()
        self.trend_plots = []
        for row, label in enumerate(("Pmax [bar]", "IMEP líq. [bar]", "CA50 [°]")):
            plot = trend.addPlot(row=row, col=0)
            plot.setLabel("left", label)
            plot.showGrid(x=True, y=True, alpha=0.3)
            if self.trend_plots:
                plot.setXLink(self.trend_plots[0])
            self.trend_plots.append(plot)
        self.trend_plots[-1].setLabel("bottom", "Ciclo (os mais recentes à direita)")
        tabs.addTab(trend, "Ciclo a ciclo")

        area.addWidget(tabs)

        self.table = QTableWidget(0, len(RESULT_COLUMNS))
        self.table.setHorizontalHeaderLabels([c[0] for c in RESULT_COLUMNS])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        area.addWidget(self.table)
        area.setStretchFactor(0, 3)
        area.setStretchFactor(1, 1)
        return area

    # Configuration

    def _geometry(self):
        return Geometry(self.bore.value(), self.stroke.value(), self.rod.value(), self.compression.value())

    def _settings(self):
        return Settings(
            gamma=self.gamma.value(),
            pegging="thermodynamic" if self.pegging.currentIndex() == 0 else "none",
            polytropic_n=self.polytropic.value())

    def _tdc_values(self):
        values = []
        for row in range(self.tdc_table.rowCount()):
            item = self.tdc_table.item(row, 1)
            try:
                values.append(float(item.text()) if item else 0.0)
            except ValueError:
                values.append(0.0)
        return values

    def _active_cylinders(self):
        return [
            self.tdc_table.item(row, 0) is not None
            and self.tdc_table.item(row, 0).checkState() == Qt.CheckState.Checked
            for row in range(self.tdc_table.rowCount())]

    def _select_cylinders(self, selected):
        self.tdc_table.blockSignals(True)
        state = Qt.CheckState.Checked if selected else Qt.CheckState.Unchecked
        for row in range(self.tdc_table.rowCount()):
            self.tdc_table.item(row, 0).setCheckState(state)
        self.tdc_table.blockSignals(False)
        self._apply_configuration()

    def _set_cylinders(self, count, tdc=None):
        self.tdc_table.blockSignals(True)
        self.tdc_table.setRowCount(count)
        self.tdc_table.setVerticalHeaderLabels([f"Cil. {i + 1}" for i in range(count)])
        for row in range(count):
            check = QTableWidgetItem()
            check.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            check.setCheckState(Qt.CheckState.Checked)
            self.tdc_table.setItem(row, 0, check)
            value = tdc[row] if tdc and row < len(tdc) else 0.0
            self.tdc_table.setItem(row, 1, QTableWidgetItem(f"{value:.1f}"))
        self.tdc_table.blockSignals(False)

        self.cylinders = count
        size = self.average_cycles.value()
        self.history = {i: deque(maxlen=size) for i in range(1, count + 1)}
        self.curves = {i: deque(maxlen=size) for i in range(1, count + 1)}
        self.table.setRowCount(count + 1)
        self.table.setVerticalHeaderLabels([f"Cil. {i + 1}" for i in range(count)] + ["Média"])
        self._create_plot_items()
        self._apply_configuration()

    def _create_plot_items(self):
        """Persistent items per cylinder and plot, updated with setData():
        main curve (mean or last cycle), min/max envelope, overlaid cycles and
        the cycle-based trends."""
        for plot in (self.p_theta, self.p_v, self.mfb_plot, self.heat_plot, *self.trend_plots):
            plot.clear()
        legend = self.p_theta.getPlotItem().legend
        if legend is not None:
            legend.clear()

        self.plot_items = {}
        for cylinder in range(1, self.cylinders + 1):
            color = pg.intColor(cylinder - 1, hues=max(self.cylinders, 1))
            pen = pg.mkPen(color, width=2)
            faint = pg.mkColor(color)
            faint.setAlpha(70)
            band = pg.mkColor(color)
            band.setAlpha(70)
            edge = pg.mkPen(band, width=1)

            items = {
                "main": (
                    self.p_theta.plot([], [], pen=pen, name=f"Cil. {cylinder}"),
                    self.p_v.plot([], [], pen=pen),
                    self.mfb_plot.plot([], [], pen=pen),
                    self.heat_plot.plot([], [], pen=pen),
                ),
                "overlay_p": [self.p_theta.plot([], [], pen=pg.mkPen(faint, width=1)) for _ in range(OVERLAY_CYCLES)],
                "overlay_q": [self.heat_plot.plot([], [], pen=pg.mkPen(faint, width=1)) for _ in range(OVERLAY_CYCLES)],
                "trend": [plot.plot([], [], pen=pg.mkPen(color, width=1), symbol="o", symbolSize=4,
                                    symbolBrush=color, symbolPen=None) for plot in self.trend_plots],
            }
            for plot, key in ((self.p_theta, "env_p"), (self.heat_plot, "env_q")):
                low = plot.plot([], [], pen=edge)
                high = plot.plot([], [], pen=edge)
                fill = pg.FillBetweenItem(low, high, brush=pg.mkBrush(band))
                plot.addItem(fill)
                items[key] = (low, high, fill)

            for item in items["main"]:
                item.setZValue(10)
            self.plot_items[cylinder] = items
        self.dirty = set(self.plot_items)

    def _apply_configuration(self, *_):
        if self.acquisition is not None:
            self.acquisition.configure(
                self._geometry(), self._settings(), self._tdc_values(), self._active_cylinders())
        # Results and curves computed with the old settings are no longer valid
        for history in (*self.history.values(), *self.curves.values()):
            history.clear()
        self._redraw()

    def _resize_history(self, size):
        self.history = {c: deque(h, maxlen=size) for c, h in self.history.items()}
        self.curves = {c: deque(h, maxlen=size) for c, h in self.curves.items()}
        self._redraw()

    def _redraw(self, *_):
        self.dirty = set(getattr(self, "plot_items", {}))

    def _load_from_simulator(self):
        info = self.acquisition.info if self.acquisition else None
        if info is None or info.cylinders == 0:
            QMessageBox.information(self, "IndiCom", "Ainda não chegaram dados do motor pelo UDP.")
            return

        for widget, value in ((self.bore, info.bore_mm), (self.stroke, info.stroke_mm),
                              (self.rod, info.rod_mm)):
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)
        if info.compression_ratio > 1:
            self.compression.blockSignals(True)
            self.compression.setValue(info.compression_ratio)
            self.compression.blockSignals(False)
        self._set_cylinders(info.cylinders, [max(t, 0.0) for t in info.tdc_deg])

    def _engine_data(self):
        return {
            "bore_mm": self.bore.value(), "stroke_mm": self.stroke.value(),
            "rod_mm": self.rod.value(), "compression_ratio": self.compression.value(),
            "tdc_deg": self._tdc_values(),
        }

    def _save_engine(self):
        ENGINES_DIR.mkdir(exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "Salvar geometria", str(ENGINES_DIR), "JSON (*.json)")
        if path:
            Path(path).write_text(json.dumps(self._engine_data(), indent=2), encoding="utf-8")

    def _open_engine(self):
        path, _ = QFileDialog.getOpenFileName(self, "Abrir geometria", str(ENGINES_DIR), "JSON (*.json)")
        if not path:
            return
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        for widget, key in ((self.bore, "bore_mm"), (self.stroke, "stroke_mm"),
                            (self.rod, "rod_mm"), (self.compression, "compression_ratio")):
            widget.blockSignals(True)
            widget.setValue(data[key])
            widget.blockSignals(False)
        self._set_cylinders(len(data["tdc_deg"]), data["tdc_deg"])

    def _connect(self):
        if self.acquisition is not None:
            self.acquisition.stop()
            self.acquisition.join(1.0)
        try:
            self.acquisition = Acquisition(self.port.value())
            self.acquisition.start()
        except OSError as error:
            self.acquisition = None
            QMessageBox.critical(
                self, "IndiCom",
                f"Não foi possível abrir a porta UDP {self.port.value()}.\n\n"
                "Provavelmente outro IndiCom já está aberto e recebendo os dados: "
                "feche esta janela ou o outro IndiCom e clique em Reconectar.\n\n"
                f"({error})")
            return
        self._apply_configuration()

    # Refresh

    def _refresh(self):
        if self.acquisition is None:
            return

        while True:
            try:
                result, curves = self.acquisition.results.get_nowait()
            except Exception:
                break
            if result.cylinder in self.history:
                self.history[result.cylinder].append(result)
                self.curves[result.cylinder].append(curves)
                self.dirty.add(result.cylinder)

        info = self.acquisition.info
        status = self.acquisition.status
        if info.cylinders:
            self.engine_label.setText(
                f"{info.name}\n{info.cylinders} cil. · {info.frequency_hz / 1000:.0f} kHz "
                f"(~{status.deg_per_sample:.2f}°/amostra)")
            if self.cylinders == 0:
                self.statusBar().showMessage("Dados do motor recebidos: use 'Carregar do simulador'", 5000)

        self.status.setText(
            f"{'Recebendo' if status.receiving else 'Sem dados'} · {status.packets_per_s:.0f} pacotes/s · "
            f"{status.rpm:.0f} rpm · pacotes perdidos: {status.lost_packets}")

        self._update_plots()
        self._update_table()

    def _update_plots(self):
        """Only cylinders with new cycles (or after a display change) are redrawn."""
        items_by_cylinder = getattr(self, "plot_items", {})
        mode = self.display_mode.currentIndex()
        for cylinder in list(self.dirty):
            items = items_by_cylinder.get(cylinder)
            if items is None:
                continue
            self._update_cylinder_plots(cylinder, items, list(self.curves.get(cylinder, [])), mode)
            self._update_trends(items, list(self.history.get(cylinder, [])))
        self.dirty.clear()

    def _update_cylinder_plots(self, cylinder, items, cycles, mode):
        p_theta, p_v, mfb, heat = items["main"]
        overlay_p, overlay_q = items["overlay_p"], items["overlay_q"]
        env_p, env_q = items["env_p"], items["env_q"]

        def clear_extra():
            for item in (*overlay_p, *overlay_q, *env_p[:2], *env_q[:2]):
                item.setData([], [])

        if not cycles:
            for item in (p_theta, p_v, mfb, heat):
                item.setData([], [])
            clear_extra()
            return

        if mode == DISPLAY_SINGLE:
            shown = cycles[-1]
        else:
            pressures = np.array([c.pressure for c in cycles])
            _, shown = mean_cycle(cylinder, pressures, self._geometry(), self._settings())

        theta = shown.theta
        p_theta.setData(theta, shown.pressure)
        positive = shown.pressure > 0
        p_v.setData(shown.volume[positive] * 1e6, shown.pressure[positive])
        valid = ~np.isnan(shown.mfb)
        mfb.setData(theta[valid], shown.mfb[valid])
        valid = ~np.isnan(shown.heat_rate)
        heat.setData(theta[valid], shown.heat_rate[valid])

        clear_extra()
        if mode == DISPLAY_ENVELOPE and len(cycles) > 1:
            env_p[0].setData(theta, pressures.min(axis=0))
            env_p[1].setData(theta, pressures.max(axis=0))
            rates = np.array([c.heat_rate for c in cycles])
            valid = ~np.isnan(rates).any(axis=0)
            env_q[0].setData(theta[valid], rates[:, valid].min(axis=0))
            env_q[1].setData(theta[valid], rates[:, valid].max(axis=0))
        elif mode == DISPLAY_OVERLAY:
            for item_p, item_q, cycle in zip(overlay_p, overlay_q, cycles[-OVERLAY_CYCLES:]):
                item_p.setData(cycle.theta, cycle.pressure)
                valid = ~np.isnan(cycle.heat_rate)
                item_q.setData(cycle.theta[valid], cycle.heat_rate[valid])

    @staticmethod
    def _update_trends(items, results):
        x = np.arange(len(results)) - len(results) + 1
        for item, attribute in zip(items["trend"], ("pmax_bar", "imep_net_bar", "ca50")):
            item.setData(x, np.array([getattr(r, attribute) for r in results], dtype=float))

    def _update_table(self):
        rows = []
        for cylinder in range(1, self.cylinders + 1):
            history = self.history.get(cylinder, [])
            rows.append(self._statistics(list(history)))
        all_results = [r for h in self.history.values() for r in h]
        rows.append(self._statistics(all_results))

        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                decimals = RESULT_COLUMNS[column][2]
                text = "—" if value is None or (isinstance(value, float) and math.isnan(value)) \
                    else f"{value:.{decimals}f}"
                item = self.table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.table.setItem(row, column, item)
                item.setText(text)

    @staticmethod
    def _statistics(results):
        fired = [r for r in results if not r.misfire]
        values = []
        for _, attribute, _ in RESULT_COLUMNS[:8]:
            data = [getattr(r, attribute) for r in fired]
            data = [d for d in data if not math.isnan(d)]
            values.append(float(np.mean(data)) if data else None)

        def cov(data):
            return float(np.std(data) / np.mean(data) * 100) if len(data) > 1 and np.mean(data) > 0 else None

        values.append(cov([r.imep_net_bar for r in results]))
        values.append(cov([r.pmax_bar for r in fired]))
        ca50 = [r.ca50 for r in fired if not math.isnan(r.ca50)]
        values.append(float(np.std(ca50)) if len(ca50) > 1 else None)
        values.append(float(len(results) - len(fired)))
        values.append(float(len(results)))
        return values

    def closeEvent(self, event):
        if self.acquisition is not None:
            self.acquisition.stop()
        super().closeEvent(event)


def lower_priority():
    """The simulator must keep real time; the analysis can wait."""
    if sys.platform == "win32":
        import ctypes
        BELOW_NORMAL_PRIORITY_CLASS = 0x00004000
        kernel32 = ctypes.windll.kernel32
        kernel32.SetPriorityClass(kernel32.GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS)


def main():
    lower_priority()
    app = QApplication(sys.argv)
    window = IndiComWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

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
from analysis import Geometry, Settings

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
    ("Falhas", None, 0),
    ("Ciclos", None, 0),
]


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
        self.curves = {}        # cylinder -> last CycleCurves

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
        form.addRow("γ (liberação de calor)", self.gamma)
        form.addRow("Pegging", self.pegging)
        form.addRow("Expoente politrópico", self.polytropic)
        form.addRow("Ciclos na média", self.average_cycles)
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
        self.history = {i: deque(maxlen=self.average_cycles.value()) for i in range(1, count + 1)}
        self.curves = {}
        self.table.setRowCount(count + 1)
        self.table.setVerticalHeaderLabels([f"Cil. {i + 1}" for i in range(count)] + ["Média"])
        self._create_plot_items()
        self._apply_configuration()

    def _create_plot_items(self):
        """One persistent curve per cylinder and plot, updated with setData()."""
        for plot in (self.p_theta, self.p_v, self.mfb_plot, self.heat_plot):
            plot.clear()
        legend = self.p_theta.getPlotItem().legend
        if legend is not None:
            legend.clear()

        self.plot_items = {}
        for cylinder in range(1, self.cylinders + 1):
            pen = pg.mkPen(pg.intColor(cylinder - 1, hues=max(self.cylinders, 1)), width=1.5)
            self.plot_items[cylinder] = (
                self.p_theta.plot([], [], pen=pen, name=f"Cil. {cylinder}"),
                self.p_v.plot([], [], pen=pen),
                self.mfb_plot.plot([], [], pen=pen),
                self.heat_plot.plot([], [], pen=pen),
            )

    def _apply_configuration(self, *_):
        if self.acquisition is not None:
            self.acquisition.configure(
                self._geometry(), self._settings(), self._tdc_values(), self._active_cylinders())
        for history in self.history.values():
            history.clear()
        active = self._active_cylinders()
        for cylinder in list(self.curves):
            if cylinder - 1 >= len(active) or not active[cylinder - 1]:
                del self.curves[cylinder]

    def _resize_history(self, size):
        self.history = {c: deque(h, maxlen=size) for c, h in self.history.items()}

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
                self.curves[result.cylinder] = curves

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
        for cylinder, (p_theta, p_v, mfb, heat) in getattr(self, "plot_items", {}).items():
            curves = self.curves.get(cylinder)
            if curves is None:
                for item in (p_theta, p_v, mfb, heat):
                    item.setData([], [])
                continue

            p_theta.setData(curves.theta, curves.pressure)
            positive = curves.pressure > 0
            p_v.setData(curves.volume[positive] * 1e6, curves.pressure[positive])
            valid = ~np.isnan(curves.mfb)
            mfb.setData(curves.theta[valid], curves.mfb[valid])
            valid = ~np.isnan(curves.heat_rate)
            heat.setData(curves.theta[valid], curves.heat_rate[valid])

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

        imep = [r.imep_net_bar for r in results]
        cov = float(np.std(imep) / np.mean(imep) * 100) if len(imep) > 1 and np.mean(imep) > 0 else None
        values.append(cov)
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

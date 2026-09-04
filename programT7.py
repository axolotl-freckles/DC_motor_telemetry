import sys
import csv
from datetime import datetime
import time
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal, QObject, Qt, QTimer
from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout,
                             QHBoxLayout, QWidget, QLabel, QSlider,
                             QTabWidget, QLineEdit, QPushButton, QGridLayout)
from PyQt6.QtGui import QDoubleValidator
import pyqtgraph as pg

from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

# --- PARAMETROS DEL MOTOR ---
R  = 6.6
L  = 0.00815
J  = 0.004
b  = 0.00132
Kt = 0.436
Kb = 0.436

# --- ESTADOS DEL FILTRO DE KALMAN EXTENDIDO (EKF) ---
Va_EKFk = 30.0
wm_EKFk = 0.0
ia_EKFk = 0.0
TL_EKFk = 0.0
P_EKF = np.diag([1.0, 1.0, 10.0, 1.0])
ekf_step_idx = 0

active_client = None
last_sent_value = -1.0

CONTROL_TICK_RATE_s = 10e-3
PACKET_SENT_RATE_s  = 10.1*CONTROL_TICK_RATE_s

def ekf_update(setpoint_rpm, rpm_raw, i_amp, torque_raw, v_raw, t_idx, data_lost=False):
    """Filtro de Kalman Extendido adaptativo - EXACTAMENTE COMO EN MATLAB"""
    global Va_EKFk, wm_EKFk, ia_EKFk, TL_EKFk, P_EKF

    n_ekf = 4
    Ts = 0.001
    datos = 100

    H_EKF = np.eye(n_ekf)
    I_EKF = np.eye(n_ekf)
    Qbase = np.diag([1e-7, 1e-5*100, 1e-5, 1e-6*100])
    Rbase = np.diag([1e-4, 5e-4, 5e-4, 1e-3])

    w_setpoint_rads = setpoint_rpm * (2.0 * np.pi / 60.0)
    w_raw_rads = rpm_raw * (2.0 * np.pi / 60.0)

    # 1. Fase de Predicción (Siempre se ejecuta, haya datos o no)
    x_pred = np.zeros(n_ekf)
    x_pred[0] = Va_EKFk
    x_pred[1] = ia_EKFk + Ts * (-(Kb / L) * wm_EKFk - (R / L) * ia_EKFk + (1 / L) * Va_EKFk)
    x_pred[2] = wm_EKFk + Ts * (-(b / J) * wm_EKFk + (Kt / J) * ia_EKFk - (1 / J) * TL_EKFk)
    x_pred[3] = TL_EKFk

    F_EKF = np.array([
        [1, 0, 0, 0],
        [Ts / L, 1 - Ts * R / L, -Ts * Kb / L, 0],
        [0, Ts * Kt / J, 1 - Ts * b / J, -Ts / J],
        [0, 0, 0, 1],
    ], dtype=float)

    e1 = w_setpoint_rads - w_raw_rads
    escala_q = 1 + 0.10 * abs(e1) + 0.50 * (np.sin(2 * np.pi * t_idx / datos) ** 2)
    escala_r = 1 + 0.25 * abs(e1) + 0.50 * (np.cos(2 * np.pi * t_idx / datos) ** 2)

    #Variables de Q y R adaptativas 
    Q_EKF = Qbase * escala_q
    R_EKF = Rbase * escala_r

    #Valores cttes promedio de Q y R (a base de prueba)
    #Q_EKF = np.diag([1.60e-7, 0.0017, 1.7e-5, 0.00017])
    #R_EKF = np.diag([0.00023, 0.0013, 0.0013, 0.0023])

    P_pred = F_EKF @ P_EKF @ F_EKF.T + Q_EKF

    # 2. Fase de Actualización (Se ignora si hay pérdida de datos)
    if data_lost:
        # El EKF confía únicamente en su modelo matemático
        x_ekf = x_pred
        P_EKFk1 = P_pred
    else:
        # Se utilizan los datos de los sensores normalmente
        z_ekf = np.array([v_raw, i_amp, w_raw_rads, torque_raw], dtype=float)
        innovacion = z_ekf - H_EKF @ x_pred
        S_EKF = H_EKF @ P_pred @ H_EKF.T + R_EKF

        try:
            S_inv = np.linalg.inv(S_EKF)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S_EKF)

        K_EKF = P_pred @ H_EKF.T @ S_inv

        x_ekf = x_pred + K_EKF @ innovacion
        P_EKFk1 = (I_EKF - K_EKF @ H_EKF) @ P_pred @ (I_EKF - K_EKF @ H_EKF).T + K_EKF @ R_EKF @ K_EKF.T

    # Guardar en memoria
    Va_EKFk = float(x_ekf[0])
    ia_EKFk = float(x_ekf[1])
    wm_EKFk = float(x_ekf[2])
    TL_EKFk = float(x_ekf[3])
    P_EKF = P_EKFk1

    rpm_est = wm_EKFk * (60.0 / (2.0 * np.pi))

    return Va_EKFk, rpm_est, ia_EKFk, TL_EKFk, np.diag(Q_EKF), np.diag(R_EKF)

# --- CONFIGURACIÓN DEL WEBSOCKET ---
class CommSignals(QObject):
    data_processed      = pyqtSignal(float,
                                     float,
                                     float,
                                     float,
                                     float,
                                     float,
                                     float,
                                     float)
    client_connected    = pyqtSignal(str)
    client_disconnected = pyqtSignal(str)

signals = CommSignals()

class ESP32WebSocketHandler(WebSocket):
    def handleMessage(self):
        if not isinstance(self.data, bytes):
            try:
                raw_string = self.data.strip()
                data_fields = [field.strip() for field in raw_string.split(',')]

                if len(data_fields) == 7:
                    received_time  = time.perf_counter()
                    timestamp      = float(data_fields[0])
                    sent_time      = float(data_fields[1])
                    setpoint       = float(data_fields[2])
                    set_voltage    = float(data_fields[3])
                    rpm            = float(data_fields[4])
                    i_amp          = float(data_fields[5])
                    estimated_load = float(data_fields[6])

                    if np.isfinite([timestamp,
                                    sent_time,
                                    received_time,
                                    setpoint,
                                    set_voltage,
                                    rpm,
                                    i_amp,
                                    estimated_load ] ).all():
                        signals.data_processed.emit(timestamp,
                                                    sent_time,
                                                    received_time,
                                                    setpoint,
                                                    set_voltage,
                                                    rpm,
                                                    i_amp,
                                                    estimated_load )
            except ValueError:
                pass

    def handleConnected(self):
        global active_client
        active_client = self
        signals.client_connected.emit(str(self.address[0]))

    def handleClose(self):
        global active_client
        active_client = None
        signals.client_disconnected.emit(str(self.address[0]))

class WebSocketServerThread(QThread):
    def __init__(self, port=8080):
        super().__init__()
        self.port = port
        self.server = None

    def run(self):
        self.server = SimpleWebSocketServer('', self.port, ESP32WebSocketHandler)
        while not self.isInterruptionRequested():
            self.server.serveonce()

    def stop(self):
        self.requestInterruption()
        if self.server:
            self.server.close()
        self.wait()

# --- INTERFAZ GRÁFICA EN PYQT6 ---
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Advanced Telemetry - EKF Only")
        self.resize(1300, 1300)

        # Activar o desactivar la grabación del archivo CSV.
        # Ponlo en False si quieres evitar el bloqueo por escritura en disco.
        self.save_csv = True

        if self.save_csv:
            timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.csv_filename = f"telemetry_log_{timestamp_str}.csv"
            self.csv_file = open(self.csv_filename, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp", "Setpoint_RPM", "ESP_Voltage", "ESP_RPM", "ESP_Current", "ESP_Torque",
                                     "EKF_Voltage", "EKF_RPM", "EKF_Current", "EKF_Torque", "Data_Lost",
                                     "Q_11", "Q_22", "Q_33", "Q_44", "R_11", "R_22", "R_33", "R_44"])
        else:
            self.csv_filename = None
            self.csv_file = None
            self.csv_writer = None

        self.max_points = 400
        self.time_data = []
        self.setpoint_data = []

        self.esp_voltage_data = []
        self.esp_rpm_data = []
        self.esp_i_amp_data = []
        self.esp_torque_data = []

        self.ekf_voltage_data = []
        self.ekf_rpm_data = []
        self.ekf_i_amp_data = []
        self.ekf_torque_data = []

        self.lost_data_state_array = []
        self.current_lost_data_state = 1.0
        self.loss_window = None
        self.loss_counter = 0

        self.last_sent_time         = None
        self.last_received_time     = None
        self.sent_time_dif_data     = []
        self.received_time_dif_data = []
        self.latency_data           = []
        self.packet_loss_data       = []
        self.q_data                 = [[] for _ in range(4)]
        self.r_data                 = [[] for _ in range(4)]

        # Bandera para activar o desactivar el perfil automático de pérdidas.
        # True = usa el perfil de pérdidas programado.
        # False = recibe toda la data normalmente.
        self.enable_loss_profile = False

        self.global_setpoint_val = 0.0
        self.is_frozen = False

        main_widget = QWidget()
        main_widget.setStyleSheet("background-color: #121212; color: white;")
        main_layout = QVBoxLayout(main_widget)

        self.status_label = QLabel("Estado: Esperando trama de telemetría...")
        self.status_label.setStyleSheet("font-weight: bold; color: #FF9800; font-size: 13px;")
        main_layout.addWidget(self.status_label)

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.South)
        self.tabs.setStyleSheet("""
            QTabBar::tab { background: #2D2D2D; color: white; padding: 10px 30px; font-weight: bold; border-radius: 4px; margin: 2px; }
            QTabBar::tab:selected { background: #007ACC; }
        """)

        pg.setConfigOption('background', '#1E1E1E')
        pg.setConfigOption('foreground', 'w')

        self.init_tab_ekf()
        self.init_tab_qr_analysis()

        main_layout.addWidget(self.tabs)
        self.setCentralWidget(main_widget)

        self.update_all_setpoint_uis(self.global_setpoint_val)

        # Transmisión
        self.transmit_timer = QTimer()
        self.transmit_timer.setInterval(60)
        self.transmit_timer.timeout.connect(self.transmit_setpoint)
        self.transmit_timer.start()

        signals.data_processed.connect(self.update_plots)
        signals.client_connected.connect(self.on_client_connect)
        signals.client_disconnected.connect(self.on_client_disconnect)

        self.server_thread = WebSocketServerThread(port=8080)
        self.server_thread.start()

    # --- MÉTODO PARA LIMPIAR GRÁFICAS ---
    def clear_graphs(self):
        """Vacía todos los arreglos de datos para limpiar las pantallas"""
        self.time_data.clear()
        self.setpoint_data.clear()

        self.esp_voltage_data.clear()
        self.esp_rpm_data.clear()
        self.esp_i_amp_data.clear()
        self.esp_torque_data.clear()

        self.ekf_voltage_data.clear()
        self.ekf_rpm_data.clear()
        self.ekf_i_amp_data.clear()
        self.ekf_torque_data.clear()

        self.lost_data_state_array.clear()

        self.received_time_dif_data.clear()
        self.sent_time_dif_data    .clear()
        self.latency_data          .clear()
        self.packet_loss_data      .clear()

        # Actualiza las curvas con listas vacías instantáneamente
        self.c_ekf_sp.setData([], [])
        self.c_ekf_rpm_est.setData([], [])
        self.c_ekf_rpm_raw.setData([], [])
        self.c_ekf_v_est.setData([], [])
        self.c_ekf_v_raw.setData([], [])
        self.c_ekf_i_est.setData([], [])
        self.c_ekf_i_raw.setData([], [])
        self.c_ekf_t_est.setData([], [])
        self.c_ekf_t_raw.setData([], [])
        self.c_lost_sig.setData([], [])
        self.c_latency_rec_diff.setData([], [])
        self.c_latency_sen_diff.setData([], [])
        self.c_latency_main    .setData([], [])
        self.c_packet_loss     .setData([], [])
        for curve in self.q_curves:
            curve.setData([], [])
        for curve in self.r_curves:
            curve.setData([], [])
        for series in self.q_data:
            series.clear()
        for series in self.r_data:
            series.clear()

    # --- PESTAÑA: EKF ---
    def init_tab_ekf(self):
        self.tab_ekf = QWidget()
        layout = QVBoxLayout(self.tab_ekf)

        top_layout = QHBoxLayout()
        self.slider_ekf = self.create_slider()
        slider_container = QHBoxLayout()
        slider_container.addWidget(QLabel("Slider Setpoint:"))
        slider_container.addWidget(self.slider_ekf)

        self.text_ekf = self.create_text_input()
        text_container = QHBoxLayout()
        text_container.addWidget(QLabel("Text Setpoint:"))
        text_container.addWidget(self.text_ekf)

        self.btn_lost_data = QPushButton("Lost Data")
        self.btn_lost_data.setCheckable(True)
        self.btn_lost_data.setStyleSheet("background-color: #E53935; color: white; font-weight: bold; padding: 5px; border-radius: 5px;")
        self.btn_lost_data.toggled.connect(self.on_lost_data_toggled)

        self.btn_reset_ekf = QPushButton("Reset Zoom")
        self.btn_reset_ekf.setStyleSheet("background-color: #555; color: white; font-weight: bold; padding: 5px 15px; border-radius: 5px;")
        self.btn_reset_ekf.clicked.connect(self.reset_zoom_ekf)

        self.btn_clear_ekf = QPushButton("Clear Plot")
        self.btn_clear_ekf.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 5px 15px; border-radius: 5px;")
        self.btn_clear_ekf.clicked.connect(self.clear_graphs)

        self.btn_freeze_ekf = QPushButton("Freeze Plot")
        self.btn_freeze_ekf.setStyleSheet("background-color: #008CBA; color: white; font-weight: bold; padding: 5px 15px; border-radius: 5px;")
        self.btn_freeze_ekf.clicked.connect(self.toggle_freeze)

        top_layout.addLayout(slider_container, stretch=2)
        top_layout.addLayout(text_container, stretch=2)
        top_layout.addWidget(self.btn_lost_data, stretch=1)
        top_layout.addWidget(self.btn_reset_ekf, stretch=1)
        top_layout.addWidget(self.btn_clear_ekf, stretch=1)
        top_layout.addWidget(self.btn_freeze_ekf, stretch=1)
        layout.addLayout(top_layout)

        # 1. Gráficas principales del EKF (Incluyendo Setpoint)
        self.g_ekf_1 = pg.PlotWidget(title="Real-Time Telemetry and Speed Control for DC Motors")
        self.g_ekf_1.addLegend(offset=(5, 5), colCount=3)  # 3 columnas para alinear las leyendas horizontalmente
        self.g_ekf_1.setYRange(0, 350)
        self.c_ekf_sp = self.g_ekf_1.plot(pen=pg.mkPen('#007ACC', width=2), name="Setpoint [RPM]")
        self.c_ekf_rpm_est = self.g_ekf_1.plot(pen=pg.mkPen('#9C27B0', width=2), name="EKF Speed [RPM]")
        self.c_ekf_rpm_raw = self.g_ekf_1.plot(pen=pg.mkPen('#4CAF50', width=1, style=Qt.PenStyle.DashLine), name="SM Speed [RPM]")

        self.g_ekf_2 = pg.PlotWidget(title="")
        self.g_ekf_2.addLegend(offset=(5, 5))
        self.g_ekf_2.setYRange(5, 30)
        self.c_ekf_v_est = self.g_ekf_2.plot(pen=pg.mkPen('#FF9800', width=2), name="EKF Voltage [V]")
        self.c_ekf_v_raw = self.g_ekf_2.plot(pen=pg.mkPen('#FF5722', width=1, style=Qt.PenStyle.DashLine), name="Armature Voltage [V]")

        self.g_ekf_3 = pg.PlotWidget(title="")
        self.g_ekf_3.addLegend(offset=(5, 5))
        self.g_ekf_3.setYRange(-1.5, 2)
        self.c_ekf_i_est = self.g_ekf_3.plot(pen=pg.mkPen('#03A9F4', width=2), name="EKF Current [A]")
        self.c_ekf_i_raw = self.g_ekf_3.plot(pen=pg.mkPen('#E91E63', width=1, style=Qt.PenStyle.DashLine), name="SM Armature Current [A]")

        self.g_ekf_4 = pg.PlotWidget(title="")
        self.g_ekf_4.addLegend(offset=(5, 5))
        self.g_ekf_4.setYRange(-1.5, 2)
        self.c_ekf_t_est = self.g_ekf_4.plot(pen=pg.mkPen('#CDDC39', width=2), name="EKF Torque [Nm]")
        self.c_ekf_t_raw = self.g_ekf_4.plot(pen=pg.mkPen('#FFC107', width=1, style=Qt.PenStyle.DashLine), name="SM Load Torque [Nm]")

        # 2. Gráfica para estado de transmisión
        self.g_lost = pg.PlotWidget()
        self.g_lost.addLegend(offset=(5, 5))
        self.g_lost.setMinimumHeight(120)
        self.g_lost.setYRange(0, 1)
        self.g_lost.getAxis('left').setTicks([[(0, '-'), (1, '-')]])
        self.c_lost_sig = self.g_lost.plot(pen=pg.mkPen('#E53935', width=2), name="Dropped/Received")

        self.g_latency = pg.PlotWidget(title="")
        self.g_latency.addLegend(offset=(5, 5))
        self.c_latency_rec_diff = self.g_latency.plot(pen=pg.mkPen( '#CDDC39', width=1, style=Qt.PenStyle.DashLine ),
                                                      name='Receive diff [s]' )
        self.c_latency_sen_diff = self.g_latency.plot(pen=pg.mkPen( '#FFC107', width=1, style=Qt.PenStyle.DashLine ),
                                                      name='Sent diff [s]' )
        self.c_latency_main     = self.g_latency.plot(pen=pg.mkPen( '#E53935', width=2 ),
                                                      name='Round Trip Time [s]')

        self.g_packet_loss = pg.PlotWidget(title="")
        self.g_packet_loss.addLegend(offset=(5, 5))
        self.c_packet_loss = self.g_packet_loss.plot(pen=pg.mkPen('#E53935', width=2),
                                                     name='Estimated packet loss')

        # 3. Formateo y distribución en dos columnas
        self.all_plots = [self.g_ekf_1,
                     self.g_ekf_2,
                     self.g_ekf_3,
                     self.g_ekf_4,
                     self.g_lost,
                     self.g_latency,
                     self.g_packet_loss ]

        for g in self.all_plots:
            g.showGrid(x=True, y=True, alpha=0.3)
            g.getAxis('left').setWidth(55)

        plots_layout = QGridLayout()
        plots_layout.setSpacing(6)

        left_plots = [self.g_ekf_1, self.g_ekf_2, self.g_ekf_3, self.g_ekf_4]
        right_plots = [self.g_lost, self.g_latency, self.g_packet_loss]

        for row, plot in enumerate(left_plots):
            plots_layout.addWidget(plot, row, 0)

        for row, plot in enumerate(right_plots):
            plots_layout.addWidget(plot, row, 1)

        plots_layout.setColumnStretch(0, 1)
        plots_layout.setColumnStretch(1, 1)
        for row in range(4):
            plots_layout.setRowStretch(row, 1)

        layout.addLayout(plots_layout)

        self.tabs.addTab(self.tab_ekf, "Extended Kalman Filter")

    def init_tab_qr_analysis(self):
        self.tab_qr = QWidget()
        layout = QVBoxLayout(self.tab_qr)

        controls_layout = QHBoxLayout()
        btn_reset_qr = QPushButton("Reset Zoom")
        btn_reset_qr.clicked.connect(self.reset_zoom_qr)
        btn_clear_qr = QPushButton("Clear Plot")
        btn_clear_qr.clicked.connect(self.clear_graphs)
        controls_layout.addWidget(btn_reset_qr)
        controls_layout.addWidget(btn_clear_qr)
        controls_layout.addStretch()
        layout.addLayout(controls_layout)

        self.g_qr = pg.PlotWidget(title="Valores de Q y R en tiempo real")
        self.g_qr.addLegend(offset=(5, 5), colCount=4)
        self.g_qr.showGrid(x=True, y=True, alpha=0.3)
        self.g_qr.getAxis('left').setWidth(55)

        q_colors = ['#FF9800', '#FFB300', '#F57C00', '#E65100']
        r_colors = ['#03A9F4', '#29B6F6', '#0288D1', '#01579B']
        self.q_curves = [
            self.g_qr.plot(
                pen=pg.mkPen(color, width=2),
                name=f'Q[{index + 1},{index + 1}]'
            )
            for index, color in enumerate(q_colors)
        ]
        self.r_curves = [
            self.g_qr.plot(
                pen=pg.mkPen(color, width=2),
                name=f'R[{index + 1},{index + 1}]'
            )
            for index, color in enumerate(r_colors)
        ]

        layout.addWidget(self.g_qr)
        self.tabs.addTab(self.tab_qr, "Q / R")

    def reset_zoom_qr(self):
        self.g_qr.enableAutoRange()
        self.g_qr.enableAutoRange(axis='x')

    def reset_zoom_ekf(self):
        self.g_ekf_1.setYRange(0, 350)
        self.g_ekf_2.setYRange(5, 30)
        self.g_ekf_3.setYRange(-1.5, 2)
        self.g_ekf_4.setYRange(-1.5, 2)
        self.g_lost.setYRange(0, 1)
        for g in self.all_plots:
            g.enableAutoRange(axis='x')

    # --- COMPONENTES Y LÓGICA DE LÍMITES (0 - 700) ---
    def create_slider(self):
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setMinimum(0)  # Mínimo permitido: 0 RPM
        slider.setMaximum(17560)
        slider.setStyleSheet("""
            QSlider::groove:horizontal { height: 8px; background: #2D2D2D; border-radius: 4px; }
            QSlider::sub-page:horizontal { background: #007ACC; border-radius: 4px; }
            QSlider::handle:horizontal { background: #FFFFFF; border: 2px solid #007ACC; width: 20px; height: 20px; margin: -6px 0; border-radius: 10px; }
        """)
        slider.valueChanged.connect(self.on_slider_changed)
        return slider

    def create_text_input(self):
        line_edit = QLineEdit()
        line_edit.setValidator(QDoubleValidator(0.0, 1756.0, 1))
        line_edit.setStyleSheet("background-color: #333; color: white; padding: 5px; border: 1px solid #555; border-radius: 3px;")
        line_edit.editingFinished.connect(self.on_text_changed)
        return line_edit

    def on_slider_changed(self):
        sender = self.sender()
        new_val = sender.value() / 10.0
        if new_val != self.global_setpoint_val:
            self.update_all_setpoint_uis(new_val, source=sender)

    def on_text_changed(self):
        sender = self.sender()
        text_val = sender.text().replace(',', '.')
        try:
            new_val = float(text_val)
            if new_val < 0.0: new_val = 0.0
            if new_val > 17560.0: new_val = 17560.0

            if new_val != self.global_setpoint_val:
                self.update_all_setpoint_uis(new_val, source=sender)
            else:
                sender.setText(f"{new_val:.1f}")
        except ValueError:
            pass

    def update_all_setpoint_uis(self, val, source=None):
        self.global_setpoint_val = val
        slider_val = int(val * 10)
        text_str = f"{val:.1f}"

        sliders = [self.slider_ekf]
        texts = [self.text_ekf]

        for s in sliders:
            if s != source:
                s.blockSignals(True)
                s.setValue(slider_val)
                s.blockSignals(False)

        for t in texts:
            if t != source:
                t.blockSignals(True)
                t.setText(text_str)
                t.blockSignals(False)

    def on_lost_data_toggled(self, checked):
        if checked:
            self.current_lost_data_state = 0.0
            self.btn_lost_data.setText("Data Lost!")
            self.btn_lost_data.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold; padding: 5px; border-radius: 5px;")
        else:
            self.current_lost_data_state = 1.0
            self.btn_lost_data.setText("Lost Data")
            self.btn_lost_data.setStyleSheet("background-color: #E53935; color: white; font-weight: bold; padding: 5px; border-radius: 5px;")

    # --- ENVÍO A ESP32 ---
    def transmit_setpoint(self):
        global active_client, last_sent_value
        current_value = self.global_setpoint_val

        if current_value != last_sent_value:
            last_sent_value = current_value
            if active_client is not None:
                try:
                    active_client.sendMessage(f"{current_value:.1f}\n")
                    print(f"[Transmisión -> ESP32]: Mandando setpoint = {current_value:.1f}")
                except Exception as e:
                    print(f"Error de transmisión: {e}")

    def on_client_connect(self, ip):
        self.status_label.setText(f"Status: ESP32 Connected ({ip}) - Receiving telemetry.")
        self.status_label.setStyleSheet("font-weight: bold; color: #4CAF50; font-size: 13px;")
        global last_sent_value
        last_sent_value = -1.0

    def on_client_disconnect(self, ip):
        self.status_label.setText("Status: ESP32 Disconnected. Searching for device...")
        self.status_label.setStyleSheet("font-weight: bold; color: #F44336; font-size: 13px;")

    # --- PERFIL AUTOMÁTICO DE VELOCIDAD Y PÉRDIDAS DE PAQUETES ---
    def get_auto_setpoint(self, timestamp):

        #return 900.0
    
        cycle = 80.0
        phase = timestamp % cycle

        if phase < 10.0:
            return 100.0
        elif phase < 60.0:
            return 900.0
        else:
            return 0.0
        
        #Standar (Copiar)
        #if phase < 10.0:
        #    return 100.0
        #elif 10 <= phase < 30.0:
        #    return 500.0
        #elif 30.0 <= phase < 50.0:
        #    return 900.0
        #elif 50.0 <= phase < 70.0:
        #    return 700.0
        #else:
        #    return 0.0

        #Prueba 1 (T=0.001)
        #if phase < 10.0:
        #    return 100.0
        #elif 10 <= phase < 30.0:
        #    return 500.0
        #elif 30.0 <= phase < 50.0:
        #    return 900.0
        #elif 50.0 <= phase < 70.0:
        #    return 700.0
        #else:
        #    return 0.0

    def should_drop_packet(self, timestamp):
        # Bandera global para activar/desactivar el perfil automático de pérdidas.
        if not self.enable_loss_profile:
            if self.loss_window is not None:
                self.loss_window = None
                self.loss_counter = 0
            return False

        # Si el perfil de pérdidas está activo, determinamos si se debe dejar caer un paquete.
        cycle = 80.0
        phase = timestamp % cycle

        if 10 <= phase < 15.0:
            loss_pattern = (2, 1) 
        elif 25.0 <= phase < 30.0:
            loss_pattern = (4, 3)
        elif 45.0 <= phase < 55.0:
            loss_pattern = (20, 18)

        #Standar (copiar)
        #if 10 <= phase < 15.0:
        #    loss_pattern = (2, 1) 
        #elif 25.0 <= phase < 30.0:
        #    loss_pattern = (4, 3)
        #elif 45.0 <= phase < 55.0:
        #    loss_pattern = (20, 18)

        #Prueba 1 (T=0.001)
        #if 10 <= phase < 15.0:
        #    loss_pattern = (2, 1) 
        #elif 25.0 <= phase < 30.0:
        #    loss_pattern = (4, 3)
        #elif 45.0 <= phase < 55.0:
        #    loss_pattern = (20, 18)
        else:
            if self.loss_window is not None:
                self.loss_window = None
                self.loss_counter = 0
            return False

        if self.loss_window != loss_pattern:
            self.loss_window = loss_pattern
            self.loss_counter = 0

        period, lost_packets = loss_pattern
        drop_now = (self.loss_counter % period) < lost_packets
        self.loss_counter += 1
        return drop_now

    # --- DIBUJADO DE GRÁFICAS Y GUARDADO EN CSV ---
    def update_plots(self,
                     timestamp,
                     sent_time,
                     received_time,
                     setpoint,
                     set_voltage,
                     rpm,
                     i_amp,
                     estimated_load ):
        # --- PERFIL AUTOMÁTICO DEL SETPOINT SEGÚN EL TIEMPO ---
        target_sp = self.get_auto_setpoint(timestamp)

        # Si el valor difiere, actualizamos los controles visuales y transmitimos
        if target_sp != self.global_setpoint_val:
            self.update_all_setpoint_uis(target_sp)

        current_sp = self.global_setpoint_val

        # Pérdidas de datos definidas por ventana temporal
        self.current_lost_data_state = 0.0 if self.should_drop_packet(timestamp) else 1.0

        self.time_data.append(timestamp)
        self.setpoint_data.append(current_sp)
        self.lost_data_state_array.append(self.current_lost_data_state)

        # Evaluar si estamos en modo "Lost Data"
        is_data_lost = (self.current_lost_data_state == 0.0)

        if is_data_lost:
            # Llenamos con NaN
            self.esp_voltage_data.append(np.nan)
            self.esp_rpm_data.append(np.nan)
            self.esp_i_amp_data.append(np.nan)
            self.esp_torque_data.append(np.nan)
            current_received_time = time.perf_counter()
            current_sent_time     = self.last_sent_time
        else:
            # Los datos llegan con normalidad
            self.esp_voltage_data.append(set_voltage)
            self.esp_rpm_data.append(rpm)
            self.esp_i_amp_data.append(i_amp)
            self.esp_torque_data.append(estimated_load)
            current_received_time = received_time
            current_sent_time     = sent_time
        # Latency calculations
        if self.last_received_time:
            received_time_dif = current_received_time - self.last_received_time
        else:
            received_time_dif = 0
        if self.last_sent_time:
            sent_time_dif = current_sent_time - self.last_sent_time
        else:
            sent_time_dif = 0
        self.last_received_time = current_received_time
        self.last_sent_time     = current_sent_time

        self.received_time_dif_data.append( received_time_dif )
        self.sent_time_dif_data    .append( sent_time_dif )
        self.latency_data.append( abs(received_time_dif - sent_time_dif) )

        estimated_packet_loss = sent_time_dif // PACKET_SENT_RATE_s

        self.packet_loss_data.append(estimated_packet_loss)

        # La magia sucede aquí: Le mandamos los datos y la bandera "is_data_lost" al EKF
        v_ekf, rpm_ekf, i_ekf, t_ekf, q_diag, r_diag = ekf_update(
            current_sp, rpm, i_amp, estimated_load, set_voltage, len(self.time_data), data_lost=is_data_lost
        )

        self.ekf_voltage_data.append(v_ekf)
        self.ekf_rpm_data.append(rpm_ekf)
        self.ekf_i_amp_data.append(i_ekf)
        self.ekf_torque_data.append(t_ekf)
        for index in range(4):
            self.q_data[index].append(float(q_diag[index]))
            self.r_data[index].append(float(r_diag[index]))

        # Guardar datos en CSV solo si la bandera lo permite.
        if self.save_csv and self.csv_writer is not None:
            self.csv_writer.writerow([
                timestamp,
                current_sp,
                set_voltage if not is_data_lost else "NaN",
                rpm if not is_data_lost else "NaN",
                i_amp if not is_data_lost else "NaN",
                estimated_load if not is_data_lost else "NaN",
                v_ekf,
                rpm_ekf,
                i_ekf,
                t_ekf,
                is_data_lost,
                q_diag[0],
                q_diag[1],
                q_diag[2],
                q_diag[3],
                r_diag[0],
                r_diag[1],
                r_diag[2],
                r_diag[3]
            ])

        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.setpoint_data.pop(0)
            self.esp_voltage_data.pop(0)
            self.esp_rpm_data.pop(0)
            self.esp_i_amp_data.pop(0)
            self.esp_torque_data.pop(0)
            self.lost_data_state_array.pop(0)

            self.ekf_voltage_data.pop(0)
            self.ekf_rpm_data.pop(0)
            self.ekf_i_amp_data.pop(0)
            self.ekf_torque_data.pop(0)
            self.received_time_dif_data.pop(0)
            self.sent_time_dif_data    .pop(0)
            self.latency_data          .pop(0)
            self.packet_loss_data      .pop(0)
            for index in range(4):
                self.q_data[index].pop(0)
                self.r_data[index].pop(0)

        if self.is_frozen:
            return  # Detiene la actualización visual sin perder datos en memoria

        # Tab EKF
        self.c_ekf_sp.setData(self.time_data, self.setpoint_data)
        self.c_ekf_rpm_est.setData(self.time_data, self.ekf_rpm_data)
        self.c_ekf_rpm_raw.setData(self.time_data, self.esp_rpm_data)
        self.c_ekf_v_est.setData(self.time_data, self.ekf_voltage_data)
        self.c_ekf_v_raw.setData(self.time_data, self.esp_voltage_data)
        self.c_ekf_i_est.setData(self.time_data, self.ekf_i_amp_data)
        self.c_ekf_i_raw.setData(self.time_data, self.esp_i_amp_data)
        self.c_ekf_t_est.setData(self.time_data, self.ekf_torque_data)
        self.c_ekf_t_raw.setData(self.time_data, self.esp_torque_data)
        self.c_lost_sig.setData(self.time_data, self.lost_data_state_array)
        self.c_latency_rec_diff.setData(self.time_data, self.received_time_dif_data)
        self.c_latency_sen_diff.setData(self.time_data, self.sent_time_dif_data)
        self.c_latency_main    .setData(self.time_data, self.latency_data)
        self.c_packet_loss     .setData(self.time_data, self.packet_loss_data)
        for index, curve in enumerate(self.q_curves):
            curve.setData(self.time_data, self.q_data[index])
        for index, curve in enumerate(self.r_curves):
            curve.setData(self.time_data, self.r_data[index])

    def closeEvent(self, event):
        # Cerrar el archivo CSV solo si se estaba usando.
        if self.save_csv and self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()
            print(f"Archivo de registro guardado como {self.csv_filename}")

        self.transmit_timer.stop()
        self.server_thread.stop()
        event.accept()

    def toggle_freeze(self):
        """Pausa o reanuda la actualización visual de las gráficas"""
        self.is_frozen = not self.is_frozen

        btn = self.btn_freeze_ekf
        if self.is_frozen:
            btn.setText("Resume Plot")
            btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 5px 15px; border-radius: 5px;")
        else:
            btn.setText("Freeze Plot")
            btn.setStyleSheet("background-color: #008CBA; color: white; font-weight: bold; padding: 5px 15px; border-radius: 5px;")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
import sys
import numpy as np
import serial
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QFormLayout,
    QDialog, QSpinBox, QPushButton, QLineEdit, QLabel
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from pyqtgraph import PlotWidget, BarGraphItem
import time

class SerialSensorReader:
    def __init__(self, port, baud_rate):
        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
    
    def get_data(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            values = line.split(',')
            if len(values) == 5:
                try:
                    return [val for val in values]
                except ValueError:
                    return None  # Em caso de erro de conversão
        return None

    def send_command(self, command):
        self.serial_port.write(f"{command}\n".encode())

class ConfigDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuração - Motion Control - GTP Nikola Tesla")

        # Centraliza o logo na tela inicial
        self.logo_label = QLabel(self)
        pixmap = QPixmap("icon_GTP.png")  # Carrega a imagem
        self.logo_label.setPixmap(pixmap)
        self.logo_label.setAlignment(Qt.AlignCenter)

        # Campos de entrada para parâmetros
        self.serial_port_input = QLineEdit()
        self.serial_port_input.setPlaceholderText("COM3 ou /dev/ttyUSB0")

        self.baud_rate_input = QSpinBox()
        self.baud_rate_input.setRange(300, 115200)
        self.baud_rate_input.setValue(115200)

        self.sampling_rate_input = QSpinBox()
        self.sampling_rate_input.setRange(10, 1000)
        self.sampling_rate_input.setValue(50)

        self.window_size_input = QSpinBox()
        self.window_size_input.setRange(100, 1000)
        self.window_size_input.setValue(500)

        # Layout do formulário
        form_layout = QFormLayout()
        form_layout.addRow("Porta Serial:", self.serial_port_input)
        form_layout.addRow("Baud Rate:", self.baud_rate_input)
        form_layout.addRow("Taxa de Amostragem (Hz):", self.sampling_rate_input)
        form_layout.addRow("Tamanho da Janela de Visualização:", self.window_size_input)

        # Botão de confirmação
        self.ok_button = QPushButton("Confirmar")
        self.ok_button.clicked.connect(self.accept)

        # Layout principal
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.logo_label)  # Adiciona o logo no layout
        main_layout.addLayout(form_layout)
        main_layout.addWidget(self.ok_button)
        self.setLayout(main_layout)

    def get_parameters(self):
        # Retorna os valores configurados pelo usuário
        return (
            self.serial_port_input.text(),
            self.baud_rate_input.value(),
            self.sampling_rate_input.value(),
            self.window_size_input.value(),
        )
from PyQt5.QtGui import QFont

class MainWindow(QMainWindow):
    def __init__(self, serial_port, baud_rate, sampling_rate, window_size):
        super().__init__()
        self.setWindowTitle("Motion Control - GTP Nikola Tesla")

        self.controller_type = "PID"

        # Variáveis de autotuning
        self.K_u = None  # Ganho crítico
        self.P_u = None  # Período crítico
        self.is_tuning = False  # Flag para indicar se estamos no processo de autotuning

        # Configuração de PID
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

        # Inicializa o temporizador de transiente e flag
        self.transient_timer = 0.0
        self.in_transient = True
        self.is_running = False

        # Inicializa o leitor serial
        self.sensor_reader = SerialSensorReader(serial_port, baud_rate)
        self.sampling_rate = sampling_rate
        self.window_size = window_size

        # Configuração dos gráficos
        self.graph_widget = PlotWidget()
        self.graph_widget.setYRange(0, 260)
        self.graph_widget.setTitle("Sinais Recebidos")
        self.graph_widget.setLabel("left", "Amplitude")
        self.graph_widget.setLabel("bottom", "Índice da Amostra")  # Eixo X como índice das amostras

        # Adiciona a legenda ao gráfico superior
        self.graph_widget.addLegend()

        self.spectrum_widget = PlotWidget()
        self.spectrum_widget.setYRange(0, 255)
        self.spectrum_widget.setTitle("Espectro de Frequência do Feedback")
        self.spectrum_widget.setLabel("left", "Magnitude")
        self.spectrum_widget.setLabel("bottom", "Frequência (Hz)")

        # Barra lateral com botões de controle
        self.create_control_buttons()

        # Layout da lateral com os labels de valores
        self.create_value_labels()

        # Layout da janela principal
        main_layout = QHBoxLayout()
        graph_layout = QVBoxLayout()
        graph_layout.addWidget(self.graph_widget)
        graph_layout.addWidget(self.spectrum_widget)
        main_layout.addLayout(graph_layout)
        main_layout.addLayout(self.control_layout)  # Adiciona a barra lateral com os controles e labels de valores

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Dados iniciais
        self.sample_indices = np.arange(-window_size,0,1)  # Índices das amostras
        self.setpoint = np.zeros(self.window_size)
        self.feedback = np.zeros(self.window_size)
        self.feedback_filtered = np.zeros(self.window_size)
        self.output_adjustment = np.zeros(self.window_size)
        
        # Frequências para o espectro
        self.freqs = np.fft.fftfreq(self.window_size, 1 / self.sampling_rate)
        self.positive_freq_indices = np.where(self.freqs > 0)[0]  # Remove a frequência 0
        self.freqs = self.freqs[self.positive_freq_indices]
        self.magnitude = np.zeros(len(self.positive_freq_indices))

        # Curvas dos gráficos
        self.setpoint_line = self.graph_widget.plot(self.sample_indices, self.setpoint, pen="b", name="Setpoint")
        self.feedback_line = self.graph_widget.plot(self.sample_indices, self.feedback, pen="y", name="Feedback")
        self.feedback_filtered_line = self.graph_widget.plot(self.sample_indices, self.feedback_filtered, pen="g", name="Feedback Filtrado")
        self.output_adjustment_line = self.graph_widget.plot(self.sample_indices, self.output_adjustment, pen="r", name="Ajuste de Saída")

        # Configuração do gráfico de barras para o espectro
        bar_width = 1.0 / len(self.freqs)  # Largura proporcional ao número de barras
        self.bar_item = BarGraphItem(x=self.freqs, height=self.magnitude, width=bar_width, brush="c")
        self.spectrum_widget.addItem(self.bar_item)

        # Configuração do temporizador para atualizar os gráficos
        self.timer = QTimer()
        self.timer.setInterval(250 // self.sampling_rate)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

    def start_autotuning(self):
        """Inicia o processo de autotuning usando o método Ziegler-Nichols."""
        if not self.is_tuning:
            self.is_tuning = True
            self.find_critical_parameters()

    def find_critical_parameters(self):
        """Encontra K_u (ganho crítico) e P_u (período crítico) usando o método de Ziegler-Nichols."""
        print("Iniciando autotuning...")

        # Passo 1: Colocar o controlador em modo proporcional (P)
        self.set_pid_values(Kp=0.000001, Ki=0.0, Kd=0.0)  # Ajuste o valor inicial de Kp
        self.send_serial_command("stop")
        time.sleep(1)
        self.send_serial_command("run")

        # Passo 2: Aumentar o Kp até o sistema começar a oscilar
        print("Aumentando o ganho até as oscilações constantes...")
        max_oscillation_amplitude = 0.1
        previous_error = None
        oscillating = False
        time_step = 0.02
        self.transient_timer = 0.0  # Resetando o tempo de transiente para começar o processo
        start_time = time.time()
        previous_time = 0

        while not oscillating and self.transient_timer < 100:  # Limite de tempo para evitar loops infinitos
            current_time = time.time()
            try:
                elapsed_time = current_time - start_time
                self.update_values()
                current_error = self.setpoint[-1] - self.feedback_filtered[-1]  # Supondo que 'feedback' e 'setpoint' são variáveis
                if previous_error is not None:
                    # Detecta se o sistema começou a oscilar (quando a diferença de erro muda de sinal repetidamente)
                    if current_error < 0:
                        oscillating = True

                if float(current_time - previous_time) > time_step:
                    # Aumenta o valor de Kp gradualmente
                    self.Kp += 0.000008
                    self.set_pid_values(Kp=self.Kp, Ki=0.0, Kd=0.0)
                    print("Kp:",self.Kp,"timestep",elapsed_time)
                    previous_error = current_error
                    
                    previous_time = current_time
                    self.transient_timer = elapsed_time
            except:
                pass
        # Passo 3: Obter K_u e P_u
        if oscillating:
            print(f"Oscilações constantes encontradas com Kp = {self.Kp}")
            self.K_u = self.Kp  # O ganho crítico é o Kp no ponto de oscilações sustentadas
            self.P_u = self.transient_timer  # O período de oscilações é o tempo de transiente
            print(f"K_u (Ganho Crítico): {self.K_u}")
            print(f"P_u (Período Crítico): {self.P_u}")
            self.calculate_pid_parameters()

    def calculate_pid_parameters(self):
        """Calcula os parâmetros PID com base nos valores de K_u e P_u usando Ziegler-Nichols."""
        if self.K_u and self.P_u:
            print("Calculando parâmetros PID usando Ziegler-Nichols...")
            
            # Fórmulas de Ziegler-Nichols para o PID
            self.Kp = 0.6 * self.K_u
            self.Ki = 2 * self.Kp / self.P_u
            self.Kd = self.Kp * self.P_u / 8

            # Exibe os parâmetros calculados
            print(f"Parâmetros PID Calculados:\nKp: {self.Kp:.4f}\nKi: {self.Ki:.4f}\nKd: {self.Kd:.4f}")
        
        else:
            print("Falha ao calcular parâmetros PID usando Ziegler-Nichols...")

        # Envia os parâmetros para o dispositivo
        self.send_pid_values_to_device()

        self.set_pid_values(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd)
        self.is_tuning = False  # Finaliza o autotuning


    def set_pid_values(self, Kp, Ki, Kd):
        """Configura os valores do controlador PID."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Aqui você pode implementar o envio dos valores para o dispositivo via serial
        command = f"set_pid {Kp} {Ki} {Kd}\n"
        self.send_serial_command(command)

    def send_pid_values_to_device(self):
        """Envia os valores PID calculados para o dispositivo via porta serial."""
        command = f"set_pid {self.Kp} {self.Ki} {self.Kd}\n"
        self.send_serial_command(command)

    def send_serial_command(self, command):
        # Envia o comando pela porta serial
        self.sensor_reader.send_command(command)

    def create_control_buttons(self):
        # Layout da barra lateral de controles
        self.control_layout = QVBoxLayout()

        # Criação do label para o logo
        self.logo_label_sidebar = QLabel(self)
        pixmap = QPixmap("icon_GTP.png")  # Carrega a imagem
        self.logo_label_sidebar.setPixmap(pixmap)
        self.logo_label_sidebar.setAlignment(Qt.AlignCenter)

        # Adiciona o logo acima dos botões
        self.control_layout.addWidget(self.logo_label_sidebar)

        # Criação dos botões
        self.start_button = QPushButton("INICIAR PROGRAMA")
        self.run_button = QPushButton("GIRA")
        self.stop_button = QPushButton("PARA")
        self.increase_speed_button = QPushButton("AUMENTAR VELOCIDADE")
        self.decrease_speed_button = QPushButton("REDUZIR VELOCIDADE")
        self.clockwise_button = QPushButton("SENTIDO DE GIRO HORÁRIO")
        self.counter_clockwise_button = QPushButton("SENTIDO DE GIRO ANTI-HORÁRIO")
        self.p_controller_button = QPushButton("CONTROLADOR P")
        self.pi_controller_button = QPushButton("CONTROLADOR PI")
        self.pid_controller_button = QPushButton("CONTROLADOR PID")

        # Novo botão para encerrar comunicação
        self.close_serial_button = QPushButton("ENCERRAR COMUNICAÇÃO")
        self.close_serial_button.clicked.connect(self.close_serial_connection)

        # Adicionando o botão "Zerar"
        self.reset_button = QPushButton("ZERAR", self)
        self.reset_button.clicked.connect(self.reset_all)

        # Conectar o botão "PARA" para resetar o tempo de transiente
        self.stop_button.clicked.connect(self.reset_transient_timer)

        # Botão para iniciar o autotuning
        self.autotune_button = QPushButton("AUTOTUNING", self)
        self.autotune_button.clicked.connect(self.start_autotuning)

        self.default_params_button = QPushButton("RESTAURAR DEFAULT")
        self.default_params_button.clicked.connect(lambda: self.restore_defaulf_params())

        # Conexão dos botões com os comandos
        self.start_button.clicked.connect(lambda: self.send_serial_command("prog0"))
        self.run_button.clicked.connect(lambda: self.start())
        self.stop_button.clicked.connect(lambda: self.stop())
        self.increase_speed_button.clicked.connect(lambda: self.send_serial_command("pwm+"))
        self.decrease_speed_button.clicked.connect(lambda: self.send_serial_command("pwm-"))
        self.clockwise_button.clicked.connect(lambda: self.send_serial_command("dir+"))
        self.counter_clockwise_button.clicked.connect(lambda: self.send_serial_command("dir-"))
        self.p_controller_button.clicked.connect(lambda: self.send_serial_command("p-control"))
        self.pi_controller_button.clicked.connect(lambda: self.send_serial_command("pi-control"))
        self.pid_controller_button.clicked.connect(lambda: self.send_serial_command("pid-control"))           

        # Adiciona os botões ao layout
        self.control_layout.addWidget(self.start_button)
        self.control_layout.addWidget(self.run_button)
        self.control_layout.addWidget(self.stop_button)
        self.control_layout.addWidget(self.increase_speed_button)
        self.control_layout.addWidget(self.decrease_speed_button)
        self.control_layout.addWidget(self.clockwise_button)
        self.control_layout.addWidget(self.counter_clockwise_button)
        self.control_layout.addWidget(self.p_controller_button)
        self.control_layout.addWidget(self.pi_controller_button)
        self.control_layout.addWidget(self.pid_controller_button)
        self.control_layout.addWidget(self.close_serial_button)  # Adiciona o botão de encerrar comunicação
        self.control_layout.addWidget(self.reset_button)
        self.control_layout.addWidget(self.autotune_button)
        self.control_layout.addWidget(self.default_params_button)
        self.control_layout.addStretch()
    
    def start(self):
        self.send_serial_command("run")
        time.sleep(0.1)
        self.in_transient = True
        self.is_running = True
        self.start_time = time.time()
    
    def stop(self):
        self.send_serial_command("stop")
        self.is_running = False

    def restore_defaulf_params(self):
        self.Kp = 0.05959
        self.Ki = 0.01511
        self.Kd = 0.02009

        self.send_pid_values_to_device()
        
        print("Parâmetros restaurados com sucesso!")
        print(f"Kp {self.Kp}")
        print(f"Ki {self.Ki}")
        print(f"Kd {self.Kd}")

    def create_value_labels(self):
        # Layout dos valores a serem exibidos
        self.value_labels_layout = QVBoxLayout()

        # Configura fonte padrão para os labels
        font = QFont()
        font.setPointSize(10)

        # Labels para os valores do setpoint, feedback, feedback filtrado e erro médio
        self.setpoint_label = QLabel("Setpoint: 0.0", self)
        self.feedback_label = QLabel("Feedback: 0.0", self)
        self.feedback_filtered_label = QLabel("Feedback Filtrado: 0.0", self)
        self.error_label = QLabel("Erro Médio: 0.0", self)
        self.controller_type_label = QLabel("Tipo de Controle: PID", self)

        # Labels para o tempo de regime, sobresinal e classificação de amortecimento
        self.settling_time_label = QLabel("Tempo de Regime: -", self)
        self.overshoot_label = QLabel("Sobresinal Máx.: -", self)
        self.damping_classification_label = QLabel("Tipo de Resposta: -", self)

        # Labels das constantes do controlador PID
        self.Kp_label = QLabel("Kp: 0.0", self)
        self.Ki_label = QLabel("Ki: 0.0", self)
        self.Kd_label = QLabel("Kd: 0.0", self)

        # Configura fontes para os labels
        for label in [self.setpoint_label, self.feedback_label, self.feedback_filtered_label,
                      self.error_label, self.controller_type_label,
                      self.settling_time_label, self.overshoot_label, self.damping_classification_label,
                      self.Kp_label,self.Ki_label,self.Kd_label]:
            label.setFont(font)

        # Adiciona os labels ao layout
        self.value_labels_layout.addWidget(self.setpoint_label)
        self.value_labels_layout.addWidget(self.feedback_label)
        self.value_labels_layout.addWidget(self.feedback_filtered_label)
        self.value_labels_layout.addWidget(self.error_label)
        self.value_labels_layout.addWidget(self.controller_type_label)
        self.value_labels_layout.addWidget(self.settling_time_label)
        self.value_labels_layout.addWidget(self.overshoot_label)
        self.value_labels_layout.addWidget(self.damping_classification_label)
        self.value_labels_layout.addWidget(self.Kp_label)
        self.value_labels_layout.addWidget(self.Ki_label)
        self.value_labels_layout.addWidget(self.Kd_label)


        # Coloca os labels na parte inferior direita, abaixo dos controles
        self.control_layout.addLayout(self.value_labels_layout)

    def send_serial_command(self, command):
        # Envia o comando serial
        self.sensor_reader.send_command(command)

    def close_serial_connection(self):
        if self.sensor_reader.serial_port.is_open:
            self.sensor_reader.serial_port.close()

    def calculate_step_response_metrics(self):
        # Obtém o valor final e máximo do feedback
        final_value = self.feedback[-1]
        peak_value = max(self.feedback)
        setpoint = self.setpoint[-1]
        current_time = time.time()
        
        # Sobresinal: diferença percentual entre o pico e o setpoint
        overshoot_percentage = ((peak_value - setpoint) / setpoint) * 100 if setpoint != 0 else 0
        
        # Tolerância para regime permanente
        tolerance = 0.075 * setpoint
        
        # Verificar se está em regime permanente
        if all(abs(self.feedback[i] - setpoint) <= tolerance for i in range(-20, 0)):
            self.in_transient = False  # Sistema estabilizado
        
        # Incrementa o temporizador de transiente enquanto o sistema não está estabilizado
        if self.in_transient:
            self.transient_timer = current_time - self.start_time
            # Determinação do tipo de amortecimento com base no sobresinal
            if overshoot_percentage > 10:
                damping_type = "Subamortecido"
            elif overshoot_percentage > 0:
                damping_type = "Criticamente Amortecido"
            else:
                damping_type = "Superamortecido"
            
            # Atualiza o label de classificação de amortecimento e o sobresinal
            self.overshoot_label.setText(f"Sobresinal Máx.: {overshoot_percentage:.2f}%")
            self.damping_classification_label.setText(f"Tipo de Resposta: {damping_type}")
        
        # Atualiza o label do tempo de transiente
        self.settling_time_label.setText(f"Tempo de Transiente: {self.transient_timer:.2f} s")

    def reset_all(self):
        # Reseta todos os valores e gráficos
        self.transient_timer = 0.0
        self.in_transient = True
        self.setpoint = np.zeros(self.window_size)
        self.feedback = np.zeros(self.window_size)
        self.feedback_filtered = np.zeros(self.window_size)
        self.output_adjustment = np.zeros(self.window_size)

        # Resetando os gráficos
        self.setpoint_line.setData(self.sample_indices, self.setpoint)
        self.feedback_line.setData(self.sample_indices, self.feedback)
        self.feedback_filtered_line.setData(self.sample_indices, self.feedback_filtered)
        self.output_adjustment_line.setData(self.sample_indices, self.output_adjustment)

        # Resetando as labels
        self.setpoint_label.setText("Setpoint: 0.00")
        self.feedback_label.setText("Feedback: 0.00")
        self.feedback_filtered_label.setText("Feedback Filtrado: 0.00")
        self.error_label.setText("Erro Médio: 0.00")
        self.controller_type_label.setText("Tipo de Controle: Nenhum")

        # Resetando as métricas de resposta
        self.overshoot_label.setText("Sobresinal Máx.: 0.00%")
        self.damping_classification_label.setText("Tipo de Resposta: Não Definido")
        self.settling_time_label.setText("Tempo de Transiente: 0.00 s")

        # Reseta o timer de transiente e a lógica
        self.transient_timer = 0.0

        # Zerando o espectro de frequências
        self.magnitude = np.zeros_like(self.magnitude)  # Limpa a magnitude do espectro
        self.bar_item.setOpts(height=self.magnitude)  # Atualiza o gráfico do espectro

    def reset_transient_timer(self):
        # Reseta o temporizador e flag de transiente
        self.transient_timer = 0.0

    def update_values(self):
        new_data = self.sensor_reader.get_data()
        if new_data is not None:
            # Unpack data and cast to float
            setpoint, feedback, feedback_filtered, output_adjustment, controller_type = new_data
            setpoint = float(setpoint)
            feedback = float(feedback)
            feedback_filtered = float(feedback_filtered)
            output_adjustment = float(output_adjustment)

            # Update the arrays and plot as before
            self.setpoint = np.roll(self.setpoint, -1)
            self.feedback = np.roll(self.feedback, -1)
            self.feedback_filtered = np.roll(self.feedback_filtered, -1)
            self.output_adjustment = np.roll(self.output_adjustment, -1)

            self.setpoint[-1] = setpoint
            self.feedback[-1] = feedback
            self.feedback_filtered[-1] = feedback_filtered
            self.output_adjustment[-1] = output_adjustment
            self.controller_type = controller_type

    def update_plots(self):
        try:
            self.update_values()
            # Update plot lines
            self.setpoint_line.setData(self.sample_indices, self.setpoint)
            self.feedback_line.setData(self.sample_indices, self.feedback)
            self.feedback_filtered_line.setData(self.sample_indices, self.feedback_filtered)
            self.output_adjustment_line.setData(self.sample_indices, self.output_adjustment)

            # Update the frequency spectrum
            self.magnitude = np.abs(np.fft.fft(self.feedback_filtered)[self.positive_freq_indices])
            self.bar_item.setOpts(height=self.magnitude)

            # Calculate average error and update labels
            error_window = self.setpoint[-20:] - self.feedback[-20:]
            avg_error = np.mean(np.abs(error_window))

            # Update labels with new values
            self.setpoint_label.setText(f"Setpoint: {self.setpoint[-1]:.2f}")
            self.feedback_label.setText(f"Feedback: {self.feedback[-1]:.2f}")
            self.feedback_filtered_label.setText(f"Feedback Filtrado: {self.feedback_filtered[-1]:.2f}")
            self.error_label.setText(f"Erro Médio: {avg_error:.2f}")
            self.controller_type_label.setText(f"Tipo de Controle: {self.controller_type}")
            self.Kp_label.setText(f"Kp: {self.Kp:.4f}")
            self.Ki_label.setText(f"Ki: {self.Ki:.4f}")
            self.Kd_label.setText(f"Kd: {self.Kd:.4f}")
            # Atualizar o cálculo das métricas de resposta de degrau
            if self.is_running is True:
                self.calculate_step_response_metrics()

        except ValueError:
            print("Data format error: unable to convert received values to float.")


def main():
    app = QApplication(sys.argv)

    # Exibe a tela de configuração
    config_dialog = ConfigDialog()
    if config_dialog.exec_() == QDialog.Accepted:
        serial_port, baud_rate, sampling_rate, window_size = config_dialog.get_parameters()

        main_window = MainWindow(serial_port, baud_rate, sampling_rate, window_size)
        main_window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

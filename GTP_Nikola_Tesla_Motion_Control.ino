// DEFINIÇÃO DE GPIO
#define START 12
#define STATUS_LED 2
#define DIRECTION_1 33
#define DIRECTION_2 25
#define STAND_BY 26
#define SPEED_FEEDBACK_SENSOR 34
#define PWM 32
#define N 3

// VARIÁVEIS
bool move = false;
bool CWdirection = false;
int counter = 0;
//int n = 100;
int speedFeedback = 0;
int speedLevel = 50;
int speedSelection = 0;
int sampleQuantity = 10000;
bool isRunning = false;
int degreeLevel = 0;
int speedErrorPercent = 0;
int speedAdjustment = 0;
float error = 0.0;
float previous_error = 0.0;
float kp = 0.05959; 
float ki = 0.01511;
float kd = 0.02009;
float p_control = 0.0;
float i_control = 0.0;
float d_control = 0.0;
String message;
String serialCommand;
String controller_type = "PID";
int sampleRate = 20; // milisegundos  x 
float h = 0.001 * sampleRate;
unsigned long previousMillis = 0;
int feedbackValues[N];
int program_time[25] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,142,144,146,148,150,152,154,156,158,160};
int program_target[25] = {50,100,75,125,200,50,75,100,125,150,175,200,225,250,100,110,120,130,140,150,160,170,180,190,200};
bool isProgramRunning = false;
int progStep = 0;
int programStartTime = 0;

// START ROTAÇÃO
void run(int direction = 0, int speed = 255){
    isRunning = true;
    serialCommand = "";
    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(DIRECTION_1,direction);
    digitalWrite(DIRECTION_2,!direction);
    digitalWrite(STAND_BY, HIGH);
    analogWrite(PWM, speed);
}

// STOP ROTAÇÃO
void stop(){
    isRunning = false;
    isProgramRunning = false;
    serialCommand = "";
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(STAND_BY, LOW);
    error = 0.0;
    previous_error = 0.0;
}

// FUNÇÃO DE CONTROLE DE VELOCIDADE
void control(){
    error = speedLevel - feedbackSignalFiltered();

    p_control = error * kp;
    i_control = i_control + (ki * h * error);
    d_control = kd * (error - previous_error) / h;

    // controle P
    if (controller_type == "P"){
      speedAdjustment = speedAdjustment + p_control;
    }

    // controle PI
    else if (controller_type == "PI"){
      speedAdjustment = speedAdjustment + p_control + i_control;
    }

    // controle PID
    else if (controller_type == "PID"){
      speedAdjustment = speedAdjustment + p_control + i_control + d_control;
    }

    previous_error = error;

    if (speedAdjustment > 255){
        speedAdjustment = 255;
    }
    
    else if (speedAdjustment < 0){
        speedAdjustment = 0;
    }
    
    else {
        analogWrite(PWM, speedAdjustment); 
    }       
}

// FUNÇÃO DE MONITORAMENTO REAL DA VELOCIDADE E CONTROLE
void monitoring(){
    /*
    Serial.print("SETPOINT:"); Serial.print(speedLevel); Serial.print(", ");
    Serial.print("FEEDBACK:"); Serial.print(speedFeedback); Serial.print(", ");
    Serial.print("FEEDBACK_FILTRADO:"); Serial.print(feedbackSignalFiltered()); Serial.print(", ");
    Serial.print("AJUSTE_SAIDA:"); Serial.print(speedAdjustment); Serial.print(", ");
    Serial.println();
    */
    Serial.print(speedLevel);
    Serial.print(",");
    Serial.print(speedFeedback);
    Serial.print(",");
    Serial.print(feedbackSignalFiltered());
    Serial.print(",");
    Serial.print(speedAdjustment);
    Serial.print(",");
    Serial.print(controller_type);
    Serial.println();
}

// Constante de suavização para o filtro EMA (quanto maior, mais suavizado)
const float alpha = 0.9;  // Ajuste entre 0 e 1. Valores menores suavizam mais.

// Variável para armazenar o valor filtrado
float filteredFeedback = 0.0;

long feedbackSignalFiltered() {
    // Aplica o filtro exponencial sobre o valor atual de feedback
    filteredFeedback = alpha * speedFeedback + (1 - alpha) * filteredFeedback;

    // Retorna o valor filtrado como um número inteiro
    return filteredFeedback;
}

void setup(){
    // mapeamento de entrada
    pinMode(START, INPUT);
    pinMode(SPEED_FEEDBACK_SENSOR, INPUT);

    // mapeamento de saída
    pinMode(DIRECTION_1, OUTPUT);
    pinMode(DIRECTION_2, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(STAND_BY, OUTPUT);

    // comunicação
    Serial.begin(115200);
}

void loop(){
    speedFeedback = abs(map(analogRead(SPEED_FEEDBACK_SENSOR), 3050,4095,0,255));
    h = 0.001 * sampleRate;
    unsigned long currentMillis = millis();
    // leitura dados recebidos na porta serial
    if (Serial.available()){
        char serialMonitor = Serial.read();

        if (serialMonitor != '\n'){
          message.concat(serialMonitor);
        }
        
        else {
          serialCommand = message;
          message = "";
        }
    }

    // COMANDO SERIAL "run"
    if (serialCommand == "run" && isRunning == false) {
        run(CWdirection, speedLevel);
        //speedAdjustment = speedLevel;
    }

    // COMANDO SERIAL "pro0"
    if (serialCommand == "prog0" && isProgramRunning == false) {
        isProgramRunning = true;
        programStartTime = currentMillis;
    }
    // COMANDO SERIAL "stop"
    else if (serialCommand == "stop" && isRunning == true){
        stop();
        speedAdjustment = 0;
    }
    // COMANDO SERIAL "pwm+"
    else if (serialCommand == "pwm+"){
        //stop();
        speedLevel = speedLevel + 25;

        if (speedLevel > 255){;
            speedLevel = 255;
        }
      
        delay(500);
        run(CWdirection, speedLevel);
    }
    // COMANDO SERIAL "pwm+5"
    else if (serialCommand == "pwm+5"){
        //stop();
        speedLevel = speedLevel + 5;

        if (speedLevel > 255){;
            speedLevel = 255;
        }

        delay(500);
        run(CWdirection, speedLevel);
    }
    // COMANDO SERIAL "pwm-"
    else if (serialCommand == "pwm-"){
        //stop();
        speedLevel = speedLevel - 25;

        if (speedLevel < 0){;
            speedLevel = 0;
        } 

        delay(500);
        run(CWdirection, speedLevel);
    }
    // COMANDO SERIAL "pwm-5"
    else if (serialCommand == "pwm-5"){
        //stop();
        speedLevel = speedLevel - 5;

        if (speedLevel < 0){;
            speedLevel = 0;
        } 

        delay(500);
        run(CWdirection, speedLevel);
    }
    // COMANDO SERIAL "dir+"
    else if (serialCommand == "dir+"){
        stop();
        CWdirection = true;
        delay(2000);
        run(CWdirection, speedLevel);
    } 
    // COMANDO SERIAL "dir-"
    else if (serialCommand == "dir-"){
        stop();
        CWdirection = false;
        delay(2000);
        run(CWdirection, speedLevel);
    }
    // COMANDO SERIAL "kp+"
    else if (serialCommand == "kp+"){
        kp += 0.001;
        serialCommand = "";
    }
    // COMANDO SERIAL "kp-"
    else if (serialCommand == "kp-"){
        kp -= 0.001;
        serialCommand = "";
    }
    // COMANDO SERIAL "ki+"
    else if (serialCommand == "ki+"){
        ki += 0.00001;
        serialCommand = "";
    }
    // COMANDO SERIAL "ki-"
    else if (serialCommand == "ki-"){
        kp -= 0.00001;
        serialCommand = "";
    }
    // COMANDO SERIAL "p-control"
    else if (serialCommand == "p-control"){
        controller_type = "P";
        serialCommand = "";
    }
    // COMANDO SERIAL "pi-control"
    else if (serialCommand == "pi-control"){
        controller_type = "PI";
        serialCommand = "";
    }
    // COMANDO SERIAL "pid-control"
    else if (serialCommand == "pid-control"){
        controller_type = "PID";
        serialCommand = "";
    }

    // COMANDO SERIAL para controle PID
    if (serialCommand.startsWith("set_pid")) {
        // Extrai os parâmetros PID enviados via serial
        // Exemplo: "set_pid 0.1 0.01 0.02"
        float new_kp, new_ki, new_kd;
        int argsParsed = sscanf(serialCommand.c_str(), "set_pid %f %f %f", &new_kp, &new_ki, &new_kd);
        if (argsParsed == 3) {
            kp = new_kp;
            ki = new_ki;
            kd = new_kd;
        }
        serialCommand = ""; // Limpar o comando serial
    }

    if (progStep == 25){
        speedLevel = program_target[0];
        run(CWdirection, speedLevel);
        progStep = 0;
        programStartTime = currentMillis;
    }
    // AÇÕES EXECUTADAS QUANDO ESTIVER EM MODO "run"
    if (currentMillis - previousMillis >= sampleRate) {
        if (isRunning) {
            monitoring();
            control();
        }
        previousMillis = currentMillis;
    }

    if (currentMillis - programStartTime >= program_time[progStep] * 250 && isProgramRunning == true){
        speedLevel = program_target[progStep];
        run(CWdirection, speedLevel);
        progStep ++;
    }
}
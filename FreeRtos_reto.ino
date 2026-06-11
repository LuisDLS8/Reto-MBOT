#include <Arduino_FreeRTOS.h>
#include <MeMegaPi.h>

//UART
#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
// Switch 2 (Interrupcion)
#define LIMIT_PIN A11

void setup(){

    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Habilita el puerto PK3 como entrada y siempre HIGH
    DDRK &= ~(1<<PK3);
    PORTK |= (1<<PK3);
    // Habilita interrupciones por cambio de pin
    PCICR |= (1 << PCIE2);
    // Habilitar PK3 (A11)
    PCMSK2 |= (1 << PCINT19);

    xTaskCreate(UART_Task,"UART",256,NULL,2,NULL);
    xTaskCreate(Motor_Task,"MOTOR",256,NULL,3,NULL);
    vTaskStartScheduler();
}

uint32_t myMillis(){
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

//Motores
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

void motor_foward_left_run(int16_t speed){
    motor_10.run(-speed);
}

void motor_foward_right_run(int16_t speed){
    motor_1.run(speed);
}

void motor_back_left_run(int16_t speed){
    motor_2.run(-speed);
}

void motor_back_right_run(int16_t speed){
    motor_9.run(speed);
}

void move_control(int16_t vx, int16_t vy, int16_t vw){
    int16_t foward_left_speed = vy + vx + vw;
    int16_t foward_right_speed = vy - vx - vw;
    int16_t back_left_speed = vy - vx + vw;
    int16_t back_right_speed = vy + vx - vw;

    if(foward_left_speed > 255){foward_left_speed = 255;}
    else if(foward_left_speed < -255){foward_left_speed = -255;}

    if(foward_right_speed > 255){foward_right_speed = 255;}
    else if(foward_right_speed < -255){foward_right_speed = -255;}

    if(back_left_speed > 255){back_left_speed = 255;}
    else if(back_left_speed < -255){back_left_speed = -255;}

    if(back_right_speed > 255){back_right_speed = 255;}
    else if(back_right_speed < -255){back_right_speed = -255;}
    
    motor_foward_left_run(foward_left_speed);
    motor_foward_right_run(foward_right_speed);
    motor_back_left_run(back_left_speed);
    motor_back_right_run(back_right_speed);
}

//Variables del comportamiento
int angulo = 0;
bool linea = false;
bool Buscar_izquierda = false;
bool Buscar_derecha = false;
uint32_t ultimoDato = 0;
volatile bool obstaculo = false;
const int velocidad = 80;
const uint32_t TIMEOUT_MS = 500;


// Interrupcion
void limitISR(){
    obstaculo = true;
}

ISR(PCINT2_vect){
    if (!(PINK & (1 << PK3))){
        limitISR();
    }
}

// 1. Tarea de Recepcion UART
void UART_Task(void *pvParameters){
    String dato = "";
    while(1){
        // Mientras que siempre este disponible la comunicacion
        while ((UCSR0A & (1 << RXC0))){
            char c = UDR0;
            if (c == '\n'){

                if (dato == "PARA"){
                    linea = false;
                    Buscar_izquierda = false;
                    Buscar_derecha = false;
                }
                else if (dato == "IZQ"){
                    linea = false;
                    Buscar_izquierda = true;
                    Buscar_derecha = false;
                }
                else if (dato == "DER"){
                    linea = false;
                    Buscar_izquierda = false;
                    Buscar_derecha = true;
                }
                else
                {
                    angulo = atoi(dato.c_str());
                    linea = true;
                    Buscar_izquierda = false;
                    Buscar_derecha = false;
                }

                ultimoDato = myMillis();
                dato = "";
            }
            else if (c != '\r'){
                dato += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 2. Tarea Control de Motores
void Motor_Task(void *pvParameters){
    for (;;){
        if (obstaculo){
    	    move_control(0, 0, 0);
    	    vTaskDelay(pdMS_TO_TICKS(100));

    	    // Giro sobre su eje
    	    move_control(0, 0, 100);

    	    vTaskDelay(pdMS_TO_TICKS(1990));

    	    // Detener giro
    	    move_control(0, 0, 0);
    	    vTaskDelay(pdMS_TO_TICKS(100));
    	    obstaculo = false;
    	    continue;
        }

        if ((myMillis() - ultimoDato) > TIMEOUT_MS){
            linea = false;
            Buscar_izquierda = false;
            Buscar_derecha = false;
        }

        if (linea){
            int vw = angulo * 2;
            if (abs(vw) < 1)
                vw = 0;
            
            if (vw > 60){vw = 60;}
            else if (vw < -60){vw = -60;}
            move_control(0,velocidad, vw);
        }
        else if (Buscar_izquierda){
            move_control(0,30,-35);
        }
        else if (Buscar_derecha){
            move_control(0,30,35);
        }
        else{
            move_control(0,0,0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void loop(){}
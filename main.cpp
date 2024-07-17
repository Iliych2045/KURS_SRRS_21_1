#include "mbed.h" // Включение основной библиотеки mbed
#include "LSM303D_my.h" // Включение библиотеки для работы с акселерометром/магнитометром
#include "lis3mdl_class.h" // Включение библиотеки для работы с магнитометром
#include "IRremouteMy.h" // Включение библиотеки для ИК-пульта
#include "TextLCD.h" // Включение библиотеки для работы с LCD дисплеем
#include "UserButton.h" // Включение библиотеки для работы с пользовательской кнопкой

#define PWM_PERIOD 100
#define MOVE_PER_L 80
#define PULSE_L (PWM_PERIOD*MOVE_PER_L)/100
#define MOVE_PER_R 80
#define PULSE_R (PWM_PERIOD*MOVE_PER_R)/100
#define ROTATE_PER_L 65
#define PULSE_ROTATE_L (PWM_PERIOD*ROTATE_PER_L)/100
#define ROTATE_PER_R 65
#define PULSE_ROTATE_R (PWM_PERIOD*ROTATE_PER_R)/100 

const int buffer_size = 255;  
typedef struct 
{
    volatile uint8_t in;   
    uint8_t event;      
    uint8_t app;
    uint8_t answer; 
    int len;
    char buffer[buffer_size];  
} RxPc;   
/*-----------------------------------------------------------------------------------
                          Prototypes                    
-----------------------------------------------------------------------------------*/
void Ini();
void Led1Ticker_isr();
void execute_pc(int events);
void ButtonIni();
void Response(uint8_t answer,char *data);
void SendResponsePc();
void Perception();
void UsDistMeasure();
void TrigTimeout_Isr();
void TrigTimeout2_Isr();
void SonarEchoRise_isr();
void SonarEchoFall_isr();
void SonarEchoRise2_isr();
void SonarEchoFall2_isr();
void SonarEchoRise3_isr();
void SonarEchoFall3_isr();
void SonarEchoFall4_isr();
void SonarEchoRise4_isr();
void MoveTout_isr();
void Stop();
void Forward();
void Backward();
void encoder_risingedge();
void RotateAzRefineTout_isr();
void RotateToAzimuth();
void WhatDelta();
void RotateL();
void RotateL_slow();
void RotateR();
void RotateR_slow();
void Rx_interrupt();
void Pc_ift_isr();
/*----------------------------------------------------------------------------------
                         Interface                        
----------------------------------------------------------------------------------*/
DigitalOut led1(LED1);
UnbufferedSerial pc(USBTX, USBRX);
I2C i2c(D14, D15);
InterruptIn button1(BUTTON1);
DigitalOut SonarTrig(A3);//Forward & Back
InterruptIn SonarEcho(A2);// Forward
InterruptIn SonarEcho3(PB_13); //Back
DigitalOut SonarTrig2(A1); //Left & Right
InterruptIn SonarEcho2(A0); //Left 
InterruptIn SonarEcho4(PB_14); //Right
//Интерфейс движения
PwmOut motor_velL(PA_8);  
PwmOut monor_dirL(PA_10); 
PwmOut motor_velR(PA_9);  
PwmOut monor_dirR(PA_11); 
InterruptIn encoder(PC_7);
/*-----------------------------------------------------------------------------------
                         Timers                           
-----------------------------------------------------------------------------------*/
Ticker Led1Ticker;
Timer SonarEchoTimer;
Timer SonarEchoTimer2;
Timer SonarEchoTimer3;
Timer SonarEchoTimer4;
Timeout TrigTimeout;
Timeout TrigTimeout2;
Timeout MoveTout;
Timeout Pc_ift;
/*-----------------------------------------------------------------------------------
                         Global variables                  
----------------------------------------------------------------------------------*/
uint8_t button_cnt;
uint8_t button_event_ind;
uint8_t button_event;
uint8_t mode;
char rx_buffer[buffer_size];
RxPc rxpc;
char tx_buffer[128];
uint8_t Rx_event;
uint8_t PC_exist=0;
uint8_t execute(char *buffer);
TextLCD_I2C lcd(&i2c,0x7e,TextLCD::LCD16x2); 
uint16_t RawSonarDist, RawSonarDist2, RawSonarDist3, RawSonarDist4;
uint8_t DistOk,Dist2Ok,Dist3Ok,Dist4Ok;
uint16_t Dist,Dist2,Dist3,Dist4;
int16_t M[3];
uint16_t heading;
uint8_t action;
uint16_t move_tout;
int odometr=0;
typedef enum 
{
    ONWARD,
    STOP,
    BACKWARD,
    ROTATE_L,
    ROTATE_R,
    ROTATE_AZIMUTH_START,
    ROTATE_AZIMUTH,
    ROTATE_AZIMUTH_REFINE,
    ROTATE_AZIMUTH_REFINE_TOUT,
    ROTATE_AZIMUTH_OK 
} state_t;
uint8_t state_az;        // Переменная для отслеживания состояния вращения по азимуту
uint16_t TargetAzimuth;  // Целевой азимут для навигации
uint8_t iteration;       // Счетчик итераций для операций, таких как уточнение азимута
int16_t delta;           // Разница между текущим и целевым азимутами
uint8_t answer_pc;
/*----------------------------------------------------------------------------------
                             Main                          
-----------------------------------------------------------------------------------*/
int main()
{
    Ini();   // Инициализировать настройки системы и периферийные устройства
    while (true) 
    {
        SendResponsePc();          // Отправить ответы через последовательный порт на ПК
        Perception();              // Обработать показания датчиков и выполнить задачи на основе окружающей среды  
        if (Rx_event)
        {
    	    rxpc.buffer[rxpc.in] = 0;
            rxpc.len = rxpc.in;
           	rxpc.in = 0;
           	execute(rxpc.buffer);
            Rx_event = 0;
        }  
    }
}

/*----------------------------------------------------------------------------------
                              Functions  
-----------------------------------------------------------------------------------*/
void Ini()
{
    pc.baud(115200);  // Установить скорость передачи данных для последовательной связи с ПК
    pc.attach(&Rx_interrupt, SerialBase::RxIrq); // Инициализировать прием данных по последовательному порту
    Rx_event = 0;
    rxpc.in = 0;
    ButtonIni(&button1);  // Инициализировать кнопку с обработчиком событий
    Led1Ticker.attach(&Led1Ticker_isr, 400000us); // Настроить таймер для переключения светодиода каждые 400 мс
    button_cnt=0;
    mode=0;
 //---------------------LCD 2x16 -------------------------------------  
    // Инициализация и настройка LCD-дисплея
    lcd.setCursor(TextLCD::CurOff_BlkOff); // Отключить курсор
    lcd.setAddress(0,0); // Установить курсор в начало первой строки
    lcd.setBacklight(TextLCD::LightOn); // Включить подсветку
    lcd.setMode(TextLCD::DispOn); // Включить дисплей
    lcd.setContrast(); // Установить контрастность (значение не указано, принято по умолчанию)
    lcd.setAddress(0,0); // Сбросить позицию курсора
    lcd.printf("Iliych2045"); // Напечатать название робота
    lcd.setAddress(0,1); // Переместить курсор на вторую строку
    lcd.printf("Start"); // Напечатать сообщение о запуске
 // Настройка ультразвуковых датчиков
    SonarEcho.fall(&SonarEchoFall_isr); // Подключить прерывание для нисходящего фронта
    SonarEcho.rise(&SonarEchoRise_isr); // Подключить прерывание для восходящего фронта
    Dist = 450; // Расстояние по умолчанию
    DistOk = 1; // Флаг, указывающий на корректное измерение расстояния
    SonarTrig = 0; // Инициализировать триггер сонара как низкий
    SonarEcho2.fall(&SonarEchoFall2_isr);
    SonarEcho2.rise(&SonarEchoRise2_isr);
    Dist2 = 450;
    Dist2Ok = 1;
    SonarTrig2 = 0;
    SonarEcho3.fall(&SonarEchoFall3_isr);
    SonarEcho3.rise(&SonarEchoRise3_isr);
    Dist3 = 450;
    Dist3Ok = 1;
    SonarEcho4.fall(&SonarEchoFall4_isr);
    SonarEcho4.rise(&SonarEchoRise4_isr);
    Dist4 = 450;
    Dist4Ok = 1; 

    // Инициализация электронного компаса LSM303D
    EcompLSM303D_Ini(&i2c);
    pc.write("Ecompass LSM303D: ID=%d", EcompLSM303D_GetID(&i2c));
    Stop(); // Убедиться, что двигатели остановлены при запуске
    
    // Установить периоды ШИМ для управления двигателями
    motor_velL.period_us(PWM_PERIOD); 
    monor_dirL.period_us(PWM_PERIOD); 
    motor_velR.period_us(PWM_PERIOD);
    monor_dirR.period_us(PWM_PERIOD);

    encoder.rise(&encoder_risingedge);// Подключить прерывание для восходящего фронта на входе энкодера
}  

// Эта функция обрабатывает команды ПК, полученные через последовательный порт
void  execute_pc(int events)
{ 
    if(events & SERIAL_EVENT_RX_CHARACTER_MATCH)
    {
        PC_exist=1;// Флаг, указывающий на активность связи с ПК
        // Разбор буфера приема от ПК в зависимости от первого символа команды
        switch(rxpc.buffer[0]) 
        {
            case 'P': //wifi sleep
            case 'p':
                break;
            case 'W': //wifi wake up
            case 'w':
                break;
            case 'T': //Test
            case 't':
                break;
            case 'A': // Команда "A" - отправка AT команд модулю Wi-Fi
                break;           
            // Выполнение команды, если она не является специальной
            default:
                if (execute(rxpc.buffer)==5)
                {
                    Response(5,tx_buffer);
                    if (tx_buffer[0]!='0')
                    {
                        pc.write(tx_buffer, strlen(tx_buffer)); 
                        rxpc.answer=0;
                        rxpc.app=0;
                    }                   
                }
                else
                    pc.write(rxpc.buffer,rxpc.len); 
                break;            
        } 
    }   
    // Повторное включение чтения из серийного порта
    pc.attach(&Rx_interrupt, SerialBase::RxIrq);
    Rx_event = 0;
    rxpc.in = 0; 
}
// Функция для выполнения команд, полученных через серийный порт
uint8_t execute(char *buffer)
{
    char *endptr;
    uint8_t answer;
    answer = 0;
    // Определение действий в зависимости от первого символа команды
    switch(buffer[0]) 
    {
        case 'S':     // Остановить движение       
        case 's': 
            answer = 6;
            Stop();
            break;
        case 'F':    // Вперёд       
        case 'f': 
            move_tout = strtol(&buffer[1],&endptr,10); // Парсинг времени движения
            answer = 1;
            Forward();
            break;
        case 'B':    // Назад++++++++++++++++      
        case 'b': 
            move_tout = strtol(&buffer[1],&endptr,10); // Парсинг времени движения
            Backward();
            answer = 2;
            break;
        // +++++++++++++++++++++++++++++++++++++++
        case 'R':    // Назад++++++++++++++++      
        case 'r': 
            move_tout = strtol(&buffer[1],&endptr,10); // Парсинг времени движения
            RotateR(); // Поворот направо
            answer = 3;
            break;
        case 'L':    // Назад++++++++++++++++      
        case 'l': 
            move_tout = strtol(&buffer[1],&endptr,10); // Парсинг времени движения
            RotateL(); // Поровот налево
            answer = 4;
            break; // Конец добавлений 
        case 'H':    // Поворот к заданному азимуту       
        case 'h':      
            TargetAzimuth = strtol(&buffer[1],&endptr,10);
            //state_az=ROTATE_AZIMUTH_START;
            RotateToAzimuth();
            answer = 5;
            break;   
        default:
            break; 
    }  
    return answer;       
}             

// Отправка ответа по серийному порту
void SendResponsePc()
{
    if(rxpc.app)
    { 
        Response(rxpc.answer,tx_buffer); 
        pc.write(tx_buffer,strlen(tx_buffer)); 
        rxpc.answer=0;
        rxpc.app=0;
    }
}           

// Функция формирования ответов на команды
void Response(uint8_t answer,char *tbuff)
{
    switch(answer) 
    {
        case 1: // Сформировать ответ типа 1
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"FORW\r\n");  // Пример формирования строкового ответа
            break;
        case  2: // Сформировать ответ типа 2
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"BACK\r\n");  // Пример формирования строкового ответа
            break;
        case 3: // Сформировать ответ типа 3
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"RGHT\r\n");  // Пример формирования строкового ответа
            break;
        case 4: // Сформировать ответ типа 4
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"LEFT\r\n");  // Пример формирования строкового ответа
            break;
        case 5: // Сформировать ответ типа 5
            //sprintf(tbuff,"Hello!\r\n"); 
            //sprintf(tbuff,"H360 D%d,%d,%d,%d\r\n",Dist,Dist2,Dist3,Dist4 );
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"RND\r\n");  // Пример формирования строкового ответа     
            break;           
        case 6:   // Сформировать ответ типа 6
            //sprintf(tbuff,"Hello!\r\n"); 
            //sprintf(tbuff,"H360 D%d,%d,%d,%d\r\n",Dist,Dist2,Dist3,Dist4 );
            sprintf(tbuff,"H%d D%d,%d,%d,%d\r\n",heading,Dist,Dist2,Dist3,Dist4 );  // Пример формирования строкового ответа
            sprintf(tbuff,"STOP\r\n");  // Пример формирования строкового ответа       
            break;
        default: 
            // Если ответ не подходит под известные типы, выводится сообщение "Nothing"
            sprintf(tbuff,"NTHG\r\n");     
            break;                          
    }
}                             

// Функция для измерения дистанции с помощью ультразвуковых датчиков
void UsDistMeasure()
{
    if(DistOk&&Dist3Ok)
    {// Если получены валидные измерения для переднего и заднего датчика
        DistOk=0;
        Dist3Ok=0;
        SonarTrig=1; // Активировать триггер переднего датчика
        TrigTimeout.attach(&TrigTimeout_Isr, 10us); // Установить тайм-аут для триггера
        // Преобразовать время эхо-сигнала в расстояние
        Dist=RawSonarDist/58;
        Dist3=RawSonarDist3/58;
    }    
    if(Dist2Ok&&Dist4Ok)
    {// Если получены валидные измерения для переднего и заднего датчика
        Dist2Ok=0;
        Dist4Ok=0;
        SonarTrig=1; // Активировать триггер переднего датчика
        TrigTimeout.attach(&TrigTimeout_Isr, 10us); // Установить тайм-аут для триггера
        // Преобразовать время эхо-сигнала в расстояние
        Dist2=RawSonarDist2/58;
        Dist4=RawSonarDist4/58;
    }
} 
// Функция обработки воспринимаемых данных и обновления информации на LCD дисплее
void Perception()
{
    UsDistMeasure();// Измерить дистанции
    lcd.setAddress(0,0); // Установить курсор в начальное положение первой строки
    lcd.printf("Y%3dX%3dy%3dx%3d",Dist,Dist2,Dist3,Dist4);// Вывод измеренных дистанций      
    //EcompLSM303D_Get_A_Axis(&i2c,A);
    EcompLSM303D_Get_M_Axis(&i2c,M); // Получить данные с магнитного датчика
    //heading = CalculateBearingCor(M[0],M[1])
    lcd.setAddress(0,1); // Перейти на вторую строку
    lcd.printf("H=%3d ",heading); // Вывод текущего направления
    lcd.setAddress(0,1); // Установка адреса повторно, возможно ошибка или особенность обновления дисплея
    lcd.printf("H%3dn%3d", heading,odometr);// Вывод направления и значения одометра
}
// Функции движения
void Stop()
{
    action = STOP; // Установить действие на остановку
    motor_velL.pulsewidth_us(0); 
    monor_dirL.pulsewidth_us(0); 
    motor_velR.pulsewidth_us(0); 
    monor_dirR.pulsewidth_us(0); 
    if (move_tout) 
    {
        MoveTout.detach(); // Отключить таймер, если он был установлен
    }  
}
void Forward ()
{
    action = ONWARD; // Установить действие на движение вперед
    motor_velL.pulsewidth_us(PULSE_ROTATE_L); // Установить ширину импульса для левого мотора
    monor_dirL.pulsewidth_us(PULSE_L); // Направление движения вперед для левого мотора
    motor_velR.pulsewidth_us(PULSE_ROTATE_R); // Ширина импульса для правого мотора выключена
    monor_dirR.pulsewidth_us(-PULSE_R); // Направление движения вперед для правого мотора
    if (move_tout) 
    {
        MoveTout.attach(&MoveTout_isr, move_tout * 1000us); // Установить таймер для автоматической остановки через заданное время 
    }   
}
void Backward()//++++++++++++++++++++++++
{
    action = BACKWARD; // Установить действие на движение назад
    motor_velL.pulsewidth_us(PULSE_ROTATE_L); // Установить ширину импульса для левого мотора
    monor_dirL.pulsewidth_us(-PULSE_L); // Направление движения назад для левого мотора
    motor_velR.pulsewidth_us(PULSE_ROTATE_R); // Ширина импульса для правого мотора выключена
    monor_dirR.pulsewidth_us(PULSE_R); // Направление дв ижения назад для правого мотора
    if (move_tout) 
    {
        MoveTout.attach(&MoveTout_isr, move_tout * 1000us); // Установить таймер для автоматической остановки через заданное время 
    }   
}
void RotateR()
{
    action = ROTATE_R; // Установить действие на поворот вправо
    motor_velR.pulsewidth_us(-PULSE_R); // Остановить правый мотор
    monor_dirR.pulsewidth_us(PULSE_ROTATE_R - 15); // Остановить правый мотор
    // ++++++++++++++++++++
    motor_velL.pulsewidth_us(PULSE_L); // Остановить правый мотор
    monor_dirL.pulsewidth_us(PULSE_ROTATE_L + 15); // Установить ширину импульса для поворота правого мотора с увеличением на 15
    if (move_tout) 
    {
        MoveTout.attach(&MoveTout_isr, move_tout * 1000us); // Установить таймер для автоматической остановки через заданное время 
    }
}
void RotateL()
{
    action = ROTATE_L; // Установить действие на поворот влево
    motor_velL.pulsewidth_us(PULSE_L); // Остановить левый мотор
    monor_dirL.pulsewidth_us(PULSE_ROTATE_L -15); // Установить ширину импульса для поворота левого мотора с увеличением на 15 
    // +++++++++++++++++++++++
    motor_velR.pulsewidth_us(-PULSE_R); // Остановить правый мотор
    monor_dirR.pulsewidth_us(PULSE_ROTATE_R + 15); // Установить ширину импульса для поворота правого мотора с увеличением на 15
    if (move_tout) 
    {
        MoveTout.attach(&MoveTout_isr, move_tout * 1000us); // Установить таймер для автоматической остановки через заданное время 
    }
}
//----------------
void RotateR_slow()
{
    action = ROTATE_R; // Установить действие на медленный поворот вправо
    motor_velL.pulsewidth_us(PULSE_ROTATE_L + 10); // Остановить левый мотор
    monor_dirL.pulsewidth_us(PULSE_L); // Уменьшить ширину импульса для замедления поворота левого мотора на 10
    // ++++++++++++++++++++++
    motor_velR.pulsewidth_us(PULSE_ROTATE_L - 10); // Остановить правый мотор
    monor_dirR.pulsewidth_us(-PULSE_R); // Установить ширину импульса для поворота правого мотора с увеличением на 15
}
void RotateL_slow()
{
    action = ROTATE_L; // Установить действие на медленный поворот влево
    motor_velL.pulsewidth_us(PULSE_ROTATE_R - 10); // Остановить левый мотор
    monor_dirL.pulsewidth_us(PULSE_L); // Уменьшить ширину импульса для замедления поворота левого мотора на 10
    // +++++++++++++++++++
    motor_velR.pulsewidth_us(PULSE_ROTATE_R + 10); // Остановить правый мотор
    monor_dirR.pulsewidth_us(-PULSE_R); // Установить ширину импульса для поворота правого мотора с увеличением на 15
}
void RotateToAzimuth()
{
    // Обработка состояния начала поворота к заданному азимуту
    switch(state_az)
    {
        case ROTATE_AZIMUTH_START:
            WhatDelta(); // Вычислить разницу между текущим и целевым азимутом
            if(delta<3)
            { 
                state_az=ROTATE_AZIMUTH_OK; // Если разница меньше 3, задача выполнена
            }
            else
            {  
                iteration=1; // Установить начальное значение итерации
                move_tout=5000; // Установить таймаут движения
                // Определение направления поворота на основе азимута
                if (heading<=90&&TargetAzimuth>=270)
                {
                    if(delta<120)
                    {    
                    }
                }
                else if (heading>=270&&TargetAzimuth<=90)
                { 
                    if(delta<120)
                    {        
                    }
                }
                else  if(heading>TargetAzimuth)
                {
                    if(delta<120)
                    {        
                    }                
                }
                else
                {          
                    if(delta<120)
                    {
                    }
                }
            }         
            break;
        case ROTATE_AZIMUTH:
            WhatDelta(); // Повторно вычислить разницу азимутов
            if (delta <= 10) 
            {
                Stop(); // Остановить движение, если разница небольшая
                state_az = ROTATE_AZIMUTH_REFINE_TOUT; // Перейти в состояние тонкой настройки
                iteration = 0; // Сбросить счетчик итераций
            }    
            break;
        case ROTATE_AZIMUTH_REFINE:
            WhatDelta(); // Повторное вычисление разницы азимутов
            if (delta < 3) 
            {
                Stop(); // Остановить движение
                // Если разница в азимутах меньше 3, останавливаем поворот
                if (iteration > 3) 
                {
                    MoveTout.detach(); // Отключаем таймер, если итераций было больше трех
                    state_az = ROTATE_AZIMUTH_OK; // Устанавливаем состояние успешного завершения поворота
                } 
                else 
                {
                    iteration++; // Увеличиваем счетчик итерации
                    state_az = ROTATE_AZIMUTH_REFINE_TOUT; // Снова устанавливаем состояние тонкой настройки
                }
            } 
            else 
            {
                // Определение направления поворота на основе сравнения текущего и целевого азимутов
                if (heading <= 90 && TargetAzimuth >= 270) 
                {
                    if (action == STOP)
                        RotateL_slow(); // Медленный поворот влево, если текущий азимут меньше 90, а целевой больше 270
                } 
                else if (heading >= 270 && TargetAzimuth <= 90) 
                {
                    // ++++++++++++++++++++++++++++++++++++
                        RotateR_slow(); // Медленный поворот вправо, если текущий азимут больше 270, а целевой меньше 90
                    // Здесь должен быть код для определенного действия
                } 
                else if (heading > TargetAzimuth) 
                {
                    // Здесь должен быть код для определенного действия
                    // ++++++++++++++++++++++++++++++++++++
                        RotateL_slow(); // Медленный поворот влево, если текущий азимут больше чем целевой
                    // Здесь должен быть код для определенного действия
                } 
                else 
                {
                    // Здесь должен быть код для определенного действия
                    // ++++++++++++++++++++++++++++++++++++++++
                        RotateR_slow();  //Медленный поворот вправо, если текущий азимут меньше чем целевой
                }
            }
            break;
        case ROTATE_AZIMUTH_OK:
            // Здесь могут быть действия или функции, вызываемые после успешного поворота
            break;
    }
}

// Функция для вычисления разницы между текущим и целевым азимутом
void WhatDelta()
{
    delta = abs((int16_t)heading - (int16_t)TargetAzimuth); // Вычисление абсолютной разницы
}

// Обработчики прерываний и таймеров
void Led1Ticker_isr()
{
    led1 = !led1; // Переключение состояния светодиода LED1
}

void TrigTimeout_Isr()
{
    SonarTrig = 0; // Сброс триггера сонара после истечения времени тайм-аута
}

void TrigTimeout2_Isr()
{
    SonarTrig2 = 0; // Сброс триггера второго сонара
}

void SonarEchoRise_isr()
{
    SonarEchoTimer.start(); // Запуск таймера при обнаружении подъема эхо-сигнала
}

void SonarEchoFall_isr()
{
    SonarEchoTimer.stop(); // Остановка таймера при обнаружении спада эхо-сигнала
    RawSonarDist = SonarEchoTimer.elapsed_time().count(); // Чтение времени, прошедшего с момента запуска таймера, для расчета дистанции
    SonarEchoTimer.reset(); // Сброс таймера
    DistOk = 1; // Установка флага валидности измерения дистанции
}

void SonarEchoRise2_isr()
{
    SonarEchoTimer2.start(); // Аналогично первому сонару для второго сонара
}
void SonarEchoFall2_isr()
{
    SonarEchoTimer2.stop(); // Аналогично первому сонару для второго сонара
    RawSonarDist2 = SonarEchoTimer2.elapsed_time().count(); // Аналогично первому сонару для второго сонара
    SonarEchoTimer2.reset(); // Аналогично первому сонару для второго сонара
    Dist2Ok = 1; // Аналогично первому сонару для второго сонара
}

void SonarEchoRise3_isr()
{
    SonarEchoTimer3.start(); // Запуск таймера при обнаружении подъема эхо-сигнала заднего сонара
}

void SonarEchoFall3_isr()
{
    SonarEchoTimer3.stop(); // Остановка таймера при обнаружении спада эхо-сигнала заднего сонара
    RawSonarDist3 = SonarEchoTimer3.elapsed_time().count(); // Чтение времени, прошедшего с момента запуска таймера, для расчета дистанции заднего сонара
    SonarEchoTimer3.reset(); // Сброс таймера
    Dist3Ok = 1; // Установка флага валидности измерения дистанции заднего сонара
}

void SonarEchoRise4_isr()
{
    SonarEchoTimer4.start(); // Запуск таймера при обнаружении подъема эхо-сигнала четвертого сонара
}

void SonarEchoFall4_isr()
{
    SonarEchoTimer4.stop(); // Остановка таймера при обнаружении спада эхо-сигнала четвертого сонара
    RawSonarDist4 = SonarEchoTimer4.elapsed_time().count(); // Чтение времени, прошедшего с момента запуска таймера, для расчета дистанции четвертого сонара
    SonarEchoTimer4.reset(); // Сброс таймера
    Dist4Ok = 1; // Установка флага валидности измерения дистанции четвертого сонара
}

void MoveTout_isr()
{
    // Этот обработчик вызывается, когда истекает таймер времени движения
    if (ROTATE_AZIMUTH_OK != state_az) // Если состояние системы не является успешным завершением поворота
        state_az = ROTATE_AZIMUTH_OK; // Установить состояние успешного завершения поворота
    Stop(); // Остановить все двигатели
}

void encoder_risingedge()
{
    odometr++; // Увеличить значение одометра при каждом подъеме сигнала от энкодера
}

void Delta()
{
    // Вычисление абсолютного значения разности между текущим и целевым азимутом
    delta = abs((int16_t)heading - (int16_t)TargetAzimuth);
}

// Callback-функции для обработки различных событий
void pc_printfCallback(char *buffer)
{
    pc.write(buffer, strlen(buffer)); // Вывод строки в серийный порт ПК
}

void Pc_ift_isr()
{
    Rx_event = 1;
}  

void Rx_interrupt()
{
    char c;
    if (pc.read(&c, 1) == 1) 
    {
    	rxpc.buffer[rxpc.in] = c; 
    	Pc_ift.attach(Pc_ift_isr, 10ms); 
    	if (Rx_event == 0 && rxpc.in<(buffer_size-1))
        {
            rxpc.in++;
        }
    } 
}
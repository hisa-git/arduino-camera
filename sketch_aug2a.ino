#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

Servo servo_x;
Servo servo_y;

const uint8_t SERVO_X_PIN = 9;
const uint8_t SERVO_Y_PIN = 10;

// Допустимый диапазон поворота сервоприводов.
const float SERVO_X_MIN = 10.0f;
const float SERVO_X_MAX = 170.0f;

const float SERVO_Y_MIN = 10.0f;
const float SERVO_Y_MAX = 170.0f;

// Начальное положение сервоприводов.
float servo_x_angle = 90.0f;
float servo_y_angle = 90.0f;

// Коэффициенты пропорционального управления.
// Чем больше значение, тем сильнее сервопривод реагирует
// на смещение объекта от центра.
const float KP_X = 0.015f;
const float KP_Y = 0.015f;

// Мёртвая зона. Если объект находится ближе этого расстояния
// к центру, сервопривод не двигается.
const float DEADZONE_X = 5.0f;
const float DEADZONE_Y = 5.0f;

// Коэффициент сглаживания входных координат.
// Значение 1.0 = сглаживания нет.
// Меньшие значения = более сильное сглаживание.
const float SMOOTHING = 0.25f;

float filtered_x = 0.0f;
float filtered_y = 0.0f;

// Буфер входящего пакета.
// Ожидаемый формат:
// x;y;area\n
//
// Например:
// -125.5;42.3;1532
char buffer[64];
uint8_t buffer_pos = 0;

void setup()
{
    Serial.begin(9600);

    // Подключаем сервоприводы к указанным пинам.
    servo_x.attach(SERVO_X_PIN);
    servo_y.attach(SERVO_Y_PIN);

    // Устанавливаем оба сервопривода в центральное положение.
    servo_x.write((int)servo_x_angle);
    servo_y.write((int)servo_y_angle);

    Serial.println("[LOG] Arduino started");
    Serial.print("[LOG] Servo X pin: ");
    Serial.println(SERVO_X_PIN);

    Serial.print("[LOG] Servo Y pin: ");
    Serial.println(SERVO_Y_PIN);

    Serial.print("[LOG] Initial X angle: ");
    Serial.println(servo_x_angle);

    Serial.print("[LOG] Initial Y angle: ");
    Serial.println(servo_y_angle);

    Serial.println("[LOG] Waiting for packets...");
}

void process_packet()
{
    // Ищем первый разделитель ';'.
    char *separator_1 = strchr(buffer, ';');

    if (separator_1 == NULL)
    {
        Serial.print("[ERR] Invalid packet: ");
        Serial.println(buffer);
        return;
    }

    // Ищем второй разделитель ';'.
    char *separator_2 = strchr(separator_1 + 1, ';');

    if (separator_2 == NULL)
    {
        Serial.print("[ERR] Invalid packet: ");
        Serial.println(buffer);
        return;
    }

    // Заменяем разделители на '\0',
    // чтобы получить три отдельные C-строки.
    *separator_1 = '\0';
    *separator_2 = '\0';

    // Формат:
    // buffer             -> X
    // separator_1 + 1    -> Y
    // separator_2 + 1    -> area
    float x = atof(buffer);
    float y = atof(separator_1 + 1);
    float area = atof(separator_2 + 1);

    // Логируем полученные данные.
    Serial.print("[DATA] X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Area=");
    Serial.println(area);

    // Экспоненциальное сглаживание координат.
    filtered_x += (x - filtered_x) * SMOOTHING;
    filtered_y += (y - filtered_y) * SMOOTHING;

    Serial.print("[FILTER] X=");
    Serial.print(filtered_x);
    Serial.print(" Y=");
    Serial.println(filtered_y);

    // Управление сервоприводом X.
    if (fabs(filtered_x) > DEADZONE_X)
    {
        float old_angle = servo_x_angle;

        servo_x_angle += filtered_x * KP_X;

        Serial.print("[SERVO X] ");
        Serial.print(old_angle);
        Serial.print(" -> ");
        Serial.println(servo_x_angle);
    }
    else
    {
        Serial.println("[SERVO X] Inside deadzone");
    }

    // Управление сервоприводом Y.
    if (fabs(filtered_y) > DEADZONE_Y)
    {
        float old_angle = servo_y_angle;

        servo_y_angle += filtered_y * KP_Y;

        Serial.print("[SERVO Y] ");
        Serial.print(old_angle);
        Serial.print(" -> ");
        Serial.println(servo_y_angle);
    }
    else
    {
        Serial.println("[SERVO Y] Inside deadzone");
    }

    // Ограничиваем углы допустимым диапазоном.
    float old_x_angle = servo_x_angle;
    float old_y_angle = servo_y_angle;

    servo_x_angle = constrain(
        servo_x_angle,
        SERVO_X_MIN,
        SERVO_X_MAX
    );

    servo_y_angle = constrain(
        servo_y_angle,
        SERVO_Y_MIN,
        SERVO_Y_MAX
    );

    // Если угол был ограничен, сообщаем об этом.
    if (old_x_angle != servo_x_angle)
    {
        Serial.print("[LIMIT] X constrained to ");
        Serial.println(servo_x_angle);
    }

    if (old_y_angle != servo_y_angle)
    {
        Serial.print("[LIMIT] Y constrained to ");
        Serial.println(servo_y_angle);
    }

    // Отправляем рассчитанные углы на сервоприводы.
    servo_x.write((int)servo_x_angle);
    servo_y.write((int)servo_y_angle);

    Serial.print("[OUTPUT] X=");
    Serial.print((int)servo_x_angle);
    Serial.print(" Y=");
    Serial.println((int)servo_y_angle);

    Serial.println("[LOG] Packet processed");
}

void loop()
{
    // Читаем все доступные байты из Serial.
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        // '\n' означает конец пакета.
        if (c == '\n')
        {
            buffer[buffer_pos] = '\0';

            // Не обрабатываем пустые строки.
            if (buffer_pos > 0)
            {
                Serial.print("[PACKET] ");
                Serial.println(buffer);

                process_packet();
            }

            // Начинаем собирать следующий пакет.
            buffer_pos = 0;
        }
        else if (c != '\r')
        {
            // Добавляем символ в буфер,
            // пока в нём есть свободное место.
            if (buffer_pos < sizeof(buffer) - 1)
            {
                buffer[buffer_pos++] = c;
            }
            else
            {
                // Пакет оказался слишком длинным.
                // Сбрасываем буфер и ждём следующий пакет.
                Serial.println("[ERR] Input buffer overflow");
                buffer_pos = 0;
            }
        }
    }
}
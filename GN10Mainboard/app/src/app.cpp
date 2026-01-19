#include "app/app.hpp"

void run()
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_Delay(100);
}
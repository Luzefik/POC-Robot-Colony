#include <stdio.h>
#include "board_config.h"


void app_main(void)
{
    printf("Ініціалізація завершена. Задачі запущені.\n");

    for (int i =0; i < 10; i++) {
        printf("Головний цикл працює: %d\n", i);
    }
}

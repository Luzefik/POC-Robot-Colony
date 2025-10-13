#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define WIDTH 320
#define HEIGHT 240

// Структура для представлення точки (або вектора)
typedef struct {
    int x;
    int y;
} Point;

// Структура для представлення об'єкта (крапки)
typedef struct {
    Point center;
    int size;
    int id;
} Blob;

// Функція для розпакування 16-бітного кольору RGB565 в 24-бітний RGB
void rgb565_to_rgb(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = (uint8_t)(((rgb565 >> 11) & 0x1F) * 255 / 31);
    *g = (uint8_t)(((rgb565 >> 5) & 0x3F) * 255 / 63);
    *b = (uint8_t)((rgb565 & 0x1F) * 255 / 31);
}

// Функція для перевірки, чи є піксель "червоним"
int is_red(uint8_t r, uint8_t g, uint8_t b) {
    // Ці пороги можна налаштувати під ваше зображення
    return (r > 180 && g < 80 && b < 80);
}

// Алгоритм пошуку в ширину (BFS) для знаходження зв'язаних компонентів (об'єктів)
void find_blobs_bfs(int x, int y, uint16_t* image_data, int* visited, int current_blob_id, Blob* blob) {
    Point queue[WIDTH * HEIGHT];
    int head = 0, tail = 0;

    queue[tail++] = (Point){x, y};
    visited[y * WIDTH + x] = current_blob_id;

    long sum_x = 0;
    long sum_y = 0;
    int pixel_count = 0;

    while (head < tail) {
        Point p = queue[head++];
        sum_x += p.x;
        sum_y += p.y;
        pixel_count++;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = p.x + dx;
                int ny = p.y + dy;

                if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT && !visited[ny * WIDTH + nx]) {
                    uint16_t pixel = image_data[ny * WIDTH + nx];
                    uint8_t r, g, b;
                    rgb565_to_rgb(pixel, &r, &g, &b);

                    if (is_red(r, g, b)) {
                        visited[ny * WIDTH + nx] = current_blob_id;
                        queue[tail++] = (Point){nx, ny};
                    }
                }
            }
        }
    }

    if (pixel_count > 0) {
        blob->center.x = sum_x / pixel_count;
        blob->center.y = sum_y / pixel_count;
        blob->size = pixel_count;
        blob->id = current_blob_id;
    }
}

// Функція для обчислення квадрату відстані між двома точками
double dist_sq(Point p1, Point p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

// Функція для перевірки колінеарності трьох точок
// Використовує площу трикутника. Якщо площа близька до нуля, точки колінеарні.
int are_collinear(Point p1, Point p2, Point p3, double tolerance) {
    long area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    return labs(area) < tolerance;
}


int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Використання: %s <файл_зображення.rdb565>\n", argv[0]);
        return 1;
    }

    const char* filename = argv[1];
    FILE* f = fopen(filename, "rb");
    if (!f) {
        perror("Не вдалося відкрити файл");
        return 1;
    }

    uint16_t* image_data = (uint16_t*)malloc(WIDTH * HEIGHT * sizeof(uint16_t));
    if (!image_data) {
        fprintf(stderr, "Не вдалося виділити пам'ять для зображення\n");
        fclose(f);
        return 1;
    }

    fread(image_data, sizeof(uint16_t), WIDTH * HEIGHT, f);
    fclose(f);

    int* visited = (int*)calloc(WIDTH * HEIGHT, sizeof(int));
    Blob* blobs = (Blob*)malloc(100 * sizeof(Blob)); // Припускаємо, що знайдемо не більше 100 об'єктів
    int blob_count = 0;

    // Шукаємо всі червоні об'єкти на зображенні
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (!visited[y * WIDTH + x]) {
                uint16_t pixel = image_data[y * WIDTH + x];
                uint8_t r, g, b;
                rgb565_to_rgb(pixel, &r, &g, &b);

                if (is_red(r, g, b)) {
                    if (blob_count < 100) {
                        find_blobs_bfs(x, y, image_data, visited, blob_count + 1, &blobs[blob_count]);
                        // Фільтруємо об'єкти за розміром
                        if (blobs[blob_count].size > 10 && blobs[blob_count].size < 200) {
                             blob_count++;
                        }
                    }
                }
            }
        }
    }

    printf("Знайдено %d червоних об'єктів (після фільтрації).\n", blob_count);
    for (int i = 0; i < blob_count; i++) {
        printf("Об'єкт %d: центр (%d, %d), розмір %d\n", i + 1, blobs[i].center.x, blobs[i].center.y, blobs[i].size);
    }
    
    // Шукаємо три точки, що задовольняють умови
    int found = 0;
    if (blob_count >= 3) {
        for (int i = 0; i < blob_count; i++) {
            for (int j = i + 1; j < blob_count; j++) {
                for (int k = j + 1; k < blob_count; k++) {
                    
                    // Перевірка на приблизно однаковий розмір
                    double size_tolerance = 0.2; // 20%
                    if (abs(blobs[i].size - blobs[j].size) / (double)blobs[i].size > size_tolerance ||
                        abs(blobs[j].size - blobs[k].size) / (double)blobs[j].size > size_tolerance) {
                        continue;
                    }
                    
                    Point p1 = blobs[i].center;
                    Point p2 = blobs[j].center;
                    Point p3 = blobs[k].center;

                    // Перевіряємо колінеарність. 
                    // Допуск (tolerance) важливий через дискретність пікселів
                    if (are_collinear(p1, p2, p3, 100.0)) {
                        double d12_sq = dist_sq(p1, p2);
                        double d23_sq = dist_sq(p2, p3);
                        double d13_sq = dist_sq(p1, p3);
                        
                        // Перевіряємо рівновіддаленість
                        double dist_tolerance = 0.1; // 10%
                        
                        // Три можливі комбінації порядку точок на лінії
                        if (fabs(sqrt(d12_sq) - sqrt(d23_sq)) < sqrt(d12_sq) * dist_tolerance && (p2.x - p1.x)*(p3.x - p2.x) >= 0) {
                             printf("\nЗнайдено три точки!\n");
                             printf("Центр 1: (%d, %d)\n", p1.x, p1.y);
                             printf("Центр 2: (%d, %d)\n", p2.x, p2.y);
                             printf("Центр 3: (%d, %d)\n", p3.x, p3.y);
                             found = 1;
                             break;
                        }
                        if (fabs(sqrt(d13_sq) - sqrt(d23_sq)) < sqrt(d13_sq) * dist_tolerance && (p3.x - p1.x)*(p2.x - p3.x) >= 0) {
                             printf("\nЗнайдено три точки!\n");
                             printf("Центр 1: (%d, %d)\n", p1.x, p1.y);
                             printf("Центр 2: (%d, %d)\n", p3.x, p3.y);
                             printf("Центр 3: (%d, %d)\n", p2.x, p2.y);
                             found = 1;
                             break;
                        }
                        if (fabs(sqrt(d12_sq) - sqrt(d13_sq)) < sqrt(d12_sq) * dist_tolerance && (p2.x - p1.x)*(p3.x - p2.x) <= 0) {
                             printf("\nЗнайдено три точки!\n");
                             printf("Центр 1: (%d, %d)\n", p3.x, p3.y);
                             printf("Центр 2: (%d, %d)\n", p1.x, p1.y);
                             printf("Центр 3: (%d, %d)\n", p2.x, p2.y);
                             found = 1;
                             break;
                        }
                    }
                }
                if (found) break;
            }
            if (found) break;
        }
    }
    
    if (!found) {
        printf("\nНе вдалося знайти три точки, що задовольняють умови.\n");
    }

    free(image_data);
    free(visited);
    free(blobs);

    return 0;
}
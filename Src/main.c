/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/*============================================================================
Name:          main.c
Author:        Emil Våhlström
Compile with:  Atollic TrueSTUDIO /
GCC: (GNU Tools for ARM Embedded Processors (Build 16.04-3)) 5.3.1
Date:          2017-02-04
Description:   Uppgift 5 - Font: Write text on a SSD1306(OLED)-screen
============================================================================*/

#include <stdlib.h>
#include "pic.h"
#include "font.h"

#define SCL(x)          HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, x)
#define SDA(x)          HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, x)
#define rSDA            HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)
#define LD2(x)          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, x)    //LD2
#define HIGH            GPIO_PIN_SET
#define LOW             GPIO_PIN_RESET
#define CDELAY          cdelay()

#define ADDRESS         (0x3c<<1)
#define WRITE           0
#define READ            1
#define HORIZONTAL      0
#define VERTICAL        1
#define COMMAND         0x00
#define DATA            0x40
#define STARTBIT        SDA(LOW);  CDELAY   //SCK=HIGH
#define STOPBIT         SCL(HIGH); CDELAY; SDA(HIGH); CDELAY; CDELAY;   //SCK=HIGH

#define CONTRAST        0x81    //A
#define DISPLAYON(x)    (0xa4+x)
#define INVERSE(x)      (0xa6+x)
#define ONOFF(x)        (0xae +x)

#define LOCOL(x)        (x)
#define HICOL(x)        (0x10+x)
#define ADDRESSING      0x20    //A
#define SETCOL          0x21    //AB
#define SETPAGE         0x22    //AB
#define SETPAGESTART(x) (0xb0+x)

#define STARTLINE(x)    (0x40+x)
#define REMAP(x)        (0xa0+x)
#define MULTIPLEX       0xa8    //A
#define OUTDIR(x)       (0xc0+(x<<3))
#define OFFSET          0xd3    //A
#define COMPINS         0xda    //A

#define DIVIDEFREQ      0xd5    //A
#define PRECHARGE       0xd9    //A
#define VCOMH           0xdb    //A
#define NOP             0xe3

#define CHARGEPUMP      0x8d    //A

#define DISABLESCROLL   0x2e

#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64
#define PAGE_HEIGHT     8

#define FONT_WIDTH      8

//k&l
#define STB(x)      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, x)         //A3
#define CLK(x)      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, x)         //A4
#define DIO(x)      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, x)         //A5
#define rDIO        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)             //A5

#define KL_DATA        0x40
#define KL_TESTMODE    0x08
#define KL_FIXED       0x04
#define KL_READ        0x02

#define KL_DISPLAY     0x80    //+ intensity
#define KL_ON          0x08

#define KL_ADDRESS     0xc0    //+ address

#define KL_LED8        0x0F

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

typedef struct Snake
{
    uint8_t x;
    uint8_t y;

    struct Snake *previous;
} Snake;

typedef struct Nibble
{
    uint8_t x;
    uint8_t y;
    uint8_t eaten;
    uint8_t state;
    uint8_t max_delay;
    uint8_t cur_delay;
} Nibble;

typedef enum SegmentNumber
{
    CLEAR = 0x00, ZERO = 0x3F, ONE = 0x06, TWO = 0x5B, THREE = 0x4F, FOUR = 0x66,
    FIVE = 0x6D, SIX = 0x7D, SEVEN = 0x07, EIGHT = 0x7F, NINE = 0x6F, A = 0x77,
    B = 0x7C, C = 0x39, D = 0x5E, E = 0x79, F = 0x71
} SegmentNumber;

typedef enum Direction { UP, RIGHT, DOWN, LEFT } Direction;

void cdelay(void);
void udelay(volatile unsigned int delay);

GPIO_PinState oled_sendbyte(uint8_t data);
GPIO_PinState oled_sendbit(GPIO_PinState val);
void oled_init(void);
void oled_clear(void);
void oled_draw(uint8_t field[64][32], Snake snake, Nibble nibble, uint8_t buffer[]);
void printstr(char *string);
void init_field(uint8_t field[64][32]);
void add_node(Snake **head);
void delete_node(Snake **head);
void init_snake(Snake **head);
void randomize_nibble(Snake snake, Nibble *nibble);
uint8_t check_collision(Snake *snake);
void restart(Snake **snake, Nibble *nibble, uint8_t *score, uint8_t highest_score, Direction *current_direction);
void change_direction(Direction *current_direction, uint8_t direction_left);
void keyled_init(void);
void keyled_send_command(uint8_t data);
void keyled_send_data(uint8_t address, uint8_t data);
void keyled_sendbyte(uint8_t data);
void keyled_printscore(uint8_t score, uint8_t base_address);
void keyled_send_led_data(uint8_t address, uint8_t data);
uint32_t keyled_readbyte(void);
uint8_t keyled_decode_input(uint32_t value);
SegmentNumber convert_to_segment(uint8_t data);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    Snake *snake;
    uint8_t field[64][32] = {0};
    uint8_t score = 0, highest_score = 0;
    uint8_t buffer[DISPLAY_WIDTH * PAGE_HEIGHT] = {0};
    uint8_t old_x , old_y, temp;
    Direction *current_direction = malloc(sizeof(Direction));
    uint8_t left_pressed = 0;
    uint8_t right_pressed = 0;
    volatile uint32_t input_data;
    volatile uint8_t keymask;
    Snake *snake_prev;

    Nibble nibble =
    {
            .x = 15,
            .y = 27,
            .eaten = 0,
            .state = 1,
            .max_delay = 10,
            .cur_delay = 0
    };

    *current_direction = RIGHT;

    init_snake(&snake);
    init_field(field);
    oled_init();
    oled_clear();
    keyled_init();
    oled_draw(field, *snake, nibble, buffer);
    keyled_printscore(score, 0x08);

    while (1)
    {
        input_data = keyled_readbyte(); //read from keys
        keymask = keyled_decode_input(input_data);

        if (left_pressed == 0 && keymask & 0x80) //LEFT KEY
        {
            left_pressed = 1;
            change_direction(current_direction, 1);
        }
        else if (left_pressed == 1 && !(keymask & 0x80)) //IF LEFT KEY RELEASED
        {
            left_pressed = 0;
        }
        if (right_pressed == 0 && keymask & 0x40) //RIGHT KEY
        {
            right_pressed = 1;
            change_direction(current_direction, 0);
        }
        else if (right_pressed == 1 && !(keymask & 0x40)) //IF RIGHT KEY RELEASED
        {
            right_pressed = 0;
        }

        if (nibble.eaten)
        {
            if (nibble.eaten == 4) //if eaten, wait randomizing nibble again
            {
                randomize_nibble(*snake, &nibble);
                nibble.eaten = 0;
            }
            else {
                nibble.eaten++;
            }
        }

        old_x = snake->x;
        old_y = snake->y;
        snake_prev = snake->previous;

        //check direction, update position of the snake's head-node
        switch (*current_direction)
        {
        case UP:
            snake->y--;
            break;
        case RIGHT:
            snake->x++;
            break;
        case DOWN:
            snake->y++;
            break;
        case LEFT:
            snake->x--;
            break;
        }

        //go through all snake-nodes and let each child get the parents position
        while (snake_prev != NULL)
        {
            if (snake_prev->x == old_x && snake_prev->y == old_y)
            {
                break;
            }
            temp = snake_prev->x;
            snake_prev->x = old_x;
            old_x = temp;
            temp = (*snake_prev).y;
            snake_prev->y = old_y;
            old_y = temp;
            snake_prev = snake_prev->previous; //giving previous node coordinates
        }

        //make sure nibble blinks every time delay >= max_delay
        if (nibble.cur_delay >= nibble.max_delay && nibble.eaten == 0)
        {
            nibble.cur_delay = 0;
            nibble.state ^= 1;
        }
        else
            nibble.cur_delay++;

        //if collision, update highscore and restart game
        if (check_collision(snake))
        {
            if (score > highest_score)
                highest_score = score;
            restart(&snake, &nibble, &score, highest_score, current_direction);
        }
        else
        {
            if (snake->x == nibble.x && snake->y == nibble.y)
            {
                for (int i = 0; i < 3; i++)
                    add_node(&snake);

                nibble.eaten = 1;
                score++;
                keyled_printscore(score, 0x08);
            }
        }

        oled_draw(field, *snake, nibble, buffer); //draw screen
        HAL_Delay(20); //put delay to not make the game to fast

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }

    /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DIO_Pin */
    GPIO_InitStruct.Pin = DIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIO_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_Pin */
    GPIO_InitStruct.Pin = CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD2_Pin SCL_Pin */
    GPIO_InitStruct.Pin = LD2_Pin|SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : STB_Pin */
    GPIO_InitStruct.Pin = STB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STB_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SDA_Pin */
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, DIO_Pin|CLK_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin|SCL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, STB_Pin|SDA_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

void cdelay(void)
{
    volatile int delay;
    for (delay=8;delay;delay--);
}

void udelay(volatile unsigned int delay)
{
    delay*=11.206;
    for (;delay;delay--);
}

GPIO_PinState oled_sendbyte(uint8_t data)
{
    GPIO_PinState ack;

    for (uint8_t i = 0x80; i != 0; i>>=1)
        oled_sendbit(!!(data & i));

    ack = oled_sendbit(HIGH); //check acknowledgement signal(should be LOW)
    SCL(LOW); //Slave releases bus, so master can write again, could also put CDELAY after
    CDELAY; //seems to work without delay, but using it to be sure that bus gets released.
    return ack;
}

GPIO_PinState oled_sendbit(GPIO_PinState val)
{
    SCL(LOW); //set clock to fall
    CDELAY; //wait for SCL to get LOW
    SDA(val); //clock is LOW, now ok to change SDA
    CDELAY; //wait for SDA to reach a stable state
    SCL(HIGH); //set clock to rise
    CDELAY; //wait for SCL to get HIGH, then data is sent to SSD1306
    return rSDA; //used in case of reading ACK
}

void oled_init(void)
{
    STOPBIT;
    STARTBIT;
    LD2(oled_sendbyte(ADDRESS+WRITE));
    LD2(oled_sendbyte(COMMAND));
    LD2(oled_sendbyte(ONOFF(0)));
    LD2(oled_sendbyte(DIVIDEFREQ));  LD2(oled_sendbyte(0x80));
    LD2(oled_sendbyte(MULTIPLEX));   LD2(oled_sendbyte(63));
    LD2(oled_sendbyte(OFFSET));      LD2(oled_sendbyte(0));
    LD2(oled_sendbyte(STARTLINE(0)));
    LD2(oled_sendbyte(CHARGEPUMP));  LD2(oled_sendbyte(0x14));
    LD2(oled_sendbyte(ADDRESSING));  LD2(oled_sendbyte(HORIZONTAL));
    LD2(oled_sendbyte(REMAP(1)));
    LD2(oled_sendbyte(OUTDIR(1)));
    LD2(oled_sendbyte(COMPINS));     LD2(oled_sendbyte(0x12));
    LD2(oled_sendbyte(CONTRAST));    LD2(oled_sendbyte(0xcf));
    LD2(oled_sendbyte(PRECHARGE));   LD2(oled_sendbyte(0xf1));
    LD2(oled_sendbyte(VCOMH));       LD2(oled_sendbyte(0x40));
    LD2(oled_sendbyte(DISPLAYON(0)));
    LD2(oled_sendbyte(INVERSE(0)));
    LD2(oled_sendbyte(DISABLESCROLL));
    LD2(oled_sendbyte(ONOFF(1)));
    LD2(oled_sendbyte(SETCOL));      LD2(oled_sendbyte(0));   LD2(oled_sendbyte(127));
    LD2(oled_sendbyte(SETPAGE));     LD2(oled_sendbyte(0));   LD2(oled_sendbyte(7));
    STOPBIT;
}

void oled_clear(void)
{
    uint16_t i, screen_size = DISPLAY_WIDTH * DISPLAY_HEIGHT;
    STARTBIT;
    LD2(oled_sendbyte(ADDRESS+WRITE));
    LD2(oled_sendbyte(DATA));
    for (i = 0; i < screen_size; i++)
        LD2(oled_sendbyte(0x00));
    STOPBIT;
}

void oled_draw(uint8_t field[64][32], Snake snake, Nibble nibble, uint8_t buffer[])
{
    volatile int x, y;
    volatile uint16_t buff_index, i;

    uint8_t display_data[64][32];

    for (x = 0; x < 64; x++)
    {
        for (y = 0; y< 32; y++)
            display_data[x][y] = field[x][y];
    }

    display_data[nibble.x][nibble.y] = nibble.state;

    Snake *snake_node = &snake;

    while (snake_node != NULL)
    {
        x = snake_node->x;
        y = snake_node->y;

        display_data[x][y] = 1;

        snake_node = snake_node->previous;
    }

    x = 0;
    y = 0;
    buff_index = 0;
    for (buff_index = 0; buff_index < (DISPLAY_WIDTH * PAGE_HEIGHT); buff_index+=2)
    {
        uint8_t segbyte =
                (display_data[x][y] << 0) | (display_data[x][y] << 1) |
                (display_data[x][y + 1] << 2) | (display_data[x][y + 1] << 3) |
                (display_data[x][y + 2] << 4) | (display_data[x][y + 2] << 5) |
                (display_data[x][y + 3] << 6) |(display_data[x][y + 3] << 7);

        buffer[buff_index] = segbyte;
        buffer[buff_index + 1] = segbyte;
        x++;

        if (x >= 64)
        {
            x = 0;
            y += 4;
        }
    }

    STARTBIT;
    LD2(oled_sendbyte(ADDRESS+WRITE));
    LD2(oled_sendbyte(DATA));
    for (i = 0; i < (DISPLAY_WIDTH * PAGE_HEIGHT); i++)
        oled_sendbyte(buffer[i]);
    STOPBIT;
}

void printstr(char *string)
{
    uint8_t i = 0, line_no;
    uint16_t offset;

    STARTBIT;
    LD2(oled_sendbyte(ADDRESS+WRITE));
    LD2(oled_sendbyte(DATA));

    while (string[i] != '\0')
    {
        offset = string[i] * FONT_WIDTH; //get offset from ascii-value
        //This loop prints out the font from left to right.
        for (line_no = 0; line_no < FONT_WIDTH; line_no++) //the font is 8 pixels wide
            LD2(oled_sendbyte(font[offset + line_no]));
        i++;
    }

    STOPBIT;
}

void init_field(uint8_t field[64][32])
{
    uint8_t x, y;
    for (x = 0; x < 64; x++)
    {
        field[x][0] = 1;
        field[x][31] = 1;

        if (x == 0)
        {
            for (y = 0; y < 32; y++)
            {
                field[0][y] = 1;
                field[63][y] = 1;
            }
        }
    }
}

void add_node(Snake **head)
{
    Snake *new_head = (Snake *)malloc(sizeof(Snake));
    new_head->x = (*head)->x;
    new_head->y = (*head)->y;
    (*head)->x = 0; //quick fix, remove later?
    (*head)->y = 0; //should do something else /w coordinates.
    new_head->previous = *head;
    *head = new_head;
}

void delete_node(Snake **head)
{
    if (*head != NULL)
    {
        Snake *temp = *head;
        *head = (*head)->previous;
        free(temp);
    }
}

void init_snake(Snake **head)
{
    *head = (Snake *)malloc(sizeof(Snake));
    (*head)->x = 4;
    (*head)->y = 27;
    (*head)->previous = NULL;

    for (int i = 0; i < 3; i++)
        add_node(head);
}

void randomize_nibble(Snake snake, Nibble *nibble)
{
    int looped = 0;
    int x, y;
    int coordinate_conflict = 0;
    do
    {
        x = 4 + (rand() % 56);
        y = 4 + (rand() % 24);

        Snake *check = &snake;

        while (check != NULL)
        {
            if (x == check->x && y == check->y)
            {
                coordinate_conflict = 1;
                break;
            }

            check = check->previous;
        }
        looped++;
        if (looped >= 128 * 64) //give up after to many tries
        {
            looped = 0;
            break;
        }
    }
    while (coordinate_conflict);

    nibble->x = x;
    nibble->y = y;
}

uint8_t check_collision(Snake *snake)
{
    int x = snake->x;
    int y = snake->y;
    Snake *current = snake->previous;

    while (current != NULL)
    {
        if (current->x == x && current->y == y)
            return 1;
        current = current->previous;
    }

    if (x == 0 || x == 63 || y == 0 || y == 31) //if border
        return 1;

    return 0;
}

void restart(Snake **snake, Nibble *nibble, uint8_t *score, uint8_t highest_score, Direction *current_direction)
{
    *score = 0;
    keyled_printscore(*score, 0x08);
    keyled_printscore(highest_score, 0x00);
    *current_direction = RIGHT;
    while ((*snake)->previous != NULL)
    {
        delete_node(snake);
    }
    init_snake(snake);

    randomize_nibble(**snake, nibble);
}

void change_direction(Direction *current_direction, uint8_t direction_left)
{
    if (direction_left)
    {
        switch (*current_direction)
        {
        case UP:
            *current_direction = LEFT;
            break;
        case RIGHT:
            *current_direction = UP;
            break;
        case DOWN:
            *current_direction = RIGHT;
            break;
        case LEFT:
            *current_direction = DOWN;
            break;
        }
    }
    else //if direction right
    {
        switch (*current_direction)
        {
        case UP:
            *current_direction = RIGHT;
            break;
        case RIGHT:
            *current_direction = DOWN;
            break;
        case DOWN:
            *current_direction = LEFT;
            break;
        case LEFT:
            *current_direction = UP;
            break;
        }
    }
}

void keyled_init(void)
{
    keyled_send_command(KL_DISPLAY | KL_ON | 0x02); //duty-cycle: (0x00: 1/16, 0x02: 4/16, 0x04: 11/16)
    keyled_send_command(KL_DATA); //set automatic address
    STB(LOW);
    CDELAY;
    keyled_sendbyte(KL_ADDRESS); //start-adress
    for (int i = 0; i < 16; i++) //clear display RAM
        keyled_sendbyte(0x00);
    STB(HIGH);
    CDELAY;
}

void keyled_send_command(uint8_t data)
{
    STB(LOW);
    CDELAY;
    keyled_sendbyte(data);
    STB(HIGH);
    CDELAY;
}

void keyled_send_data(uint8_t address, uint8_t data)
{
    STB(LOW); //begin command
    CDELAY;
    keyled_sendbyte(address);
    keyled_sendbyte(data);
    STB(HIGH); //end transmission
    CDELAY;
}

void keyled_sendbyte(uint8_t data)
{
    uint8_t i;

    //falling edge: master send -> rising edge: slave read
    for(i=0x1;i!=0;i<<=1)
    {
        DIO(!!(data & i)); //!! flips bits above 1 to value 1
        CLK(LOW); //DIO outputs data at the falling edge.
        CDELAY; //put delay between pulses
        CLK(HIGH);
        CDELAY; //put delay between pulses
    }
}

void keyled_printscore(uint8_t score, uint8_t base_address)
{
    uint8_t hundreds = 0, tens = 0, ones = 0;
    uint8_t segment_dec_low, segment_dec_mid, segment_dec_high;
    hundreds = score / 100;
    tens = (score % 100) / 10;
    ones = score % 10;

    segment_dec_low = convert_to_segment(ones);
    segment_dec_mid = convert_to_segment(tens);
    segment_dec_high = convert_to_segment(hundreds);

    keyled_send_led_data(KL_ADDRESS | (base_address + 0x06), segment_dec_low);
    keyled_send_led_data(KL_ADDRESS | (base_address + 0x04), segment_dec_mid);
    keyled_send_led_data(KL_ADDRESS | (base_address + 0x02), segment_dec_high);
}

void keyled_send_led_data(uint8_t address, uint8_t data)
{
    STB(LOW); //begin command
    CDELAY;
    keyled_sendbyte(address);
    keyled_sendbyte(data);
    STB(HIGH); //end transmission
    CDELAY;
}

uint32_t keyled_readbyte(void)
{
    uint32_t i, data = 0x0000;

    STB(LOW); //begin command by lowering STB
    CDELAY; //according to timing diagram wait before sending data
    keyled_sendbyte(KL_DATA | KL_READ);
    DIO(HIGH); //to read data on open drain, OD needs a power voltage that it can lead to ground.
    CDELAY; //wait before reading data

    //falling edge: slave send -> rising edge: master read
    for (i = 0; i!=32; i++)
    {
        CLK(LOW); //data output from slave is read into DIO
        CDELAY; //put delay between pulses
        CLK(HIGH); //now okay for slave to input data again
        data |= (rDIO << i); //read on positive clock-flank
        CDELAY; //put delay between pulses
    }

    STB(HIGH); //end transmission
    CDELAY; //wait before next transmission

    return data;
}

uint8_t keyled_decode_input(uint32_t value)
{
    return
            (value & 0x00000001)<<7  |
            (value & 0x00000100)>>2  |
            (value & 0x00010000)>>11 |
            (value & 0x01000000)>>20 |
            (value & 0x00000010)>>1  |
            (value & 0x00001000)>>10 |
            (value & 0x00100000)>>19 |
            (value & 0x10000000)>>28;
}

SegmentNumber convert_to_segment(uint8_t data)
{
    uint8_t nibble = 0x0F & data; //erase bit 4-7
    switch(nibble)
    {
    case (0x01): return ONE;
    case (0x02): return TWO;
    case (0x03): return THREE;
    case (0x04): return FOUR;
    case (0x05): return FIVE;
    case (0x06): return SIX;
    case (0x07): return SEVEN;
    case (0x08): return EIGHT;
    case (0x09): return NINE;
    case (0x0A): return A;
    case (0x0B): return B;
    case (0x0C): return C;
    case (0x0D): return D;
    case (0x0E): return E;
    case (0x0F): return F;
    default: return ZERO;
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

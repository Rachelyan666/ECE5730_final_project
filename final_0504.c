/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/divider.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

// === the fixed point macros ========================================
// Wall detection
#define hitBottom(b) (b > int2fix15(380))
#define hitTop(b) (b < int2fix15(100))
#define hitLeft(a) (a < int2fix15(100))
#define hitRight(a) (a > int2fix15(380))

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE;
char screentext[40];

// msgs from uart
volatile int received_num1;
volatile int received_num2;

#define INPUT_BUFFER_SIZE 1

// character array
char screentext[40];

// draw speed
int threshold = 1;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// PWM configuration constants
#define PWM_OUT_1 4 // GPIO pin used for PWM output
#define PWM_OUT_2 5 // GPIO pin used for PWM output

#define PWM_FREQ_HZ 50 // Servo expects 50Hz signal
#define PERIOD_US (1000000 / PWM_FREQ_HZ)
#define CLKDIV 64.0f // PWM clock divider
#define TIME_CLK_HZ (125000000 / CLKDIV)
#define WRAPVAL (TIME_CLK_HZ / PWM_FREQ_HZ) // 125MHz / 64 = ~1.953 MHz; 1.953 MHz / 50Hz = ~39062

#define MIN_PULSE_US 500  // 0° angle pulse (µs)
#define MAX_PULSE_US 2500 // 180° angle pulse (µs)
// #define MAX_ANGLE 270
// #define MIN_ANGLE 0
#define MAX_ANGLE 165
#define MIN_ANGLE 85

// UART configurations
#define UART_ID1 uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define ubuf_size 64

// Some paramters for PWM
uint slice_num;
volatile fix15 control1 = int2fix15(135);    // Initial servo angle
volatile fix15 old_control1 = int2fix15(-1); // Track last angle to update only on change
volatile fix15 control2 = int2fix15(135);    // Initial servo angle
volatile fix15 old_control2 = int2fix15(-1); // Track last angle to update only on change

// PID stuff
float x_error;
float y_error;
static int new_error;

int time_up = 0;
#define DER_UPDATE 1
fix15 error_fx = 0;
fix15 error_fy = 0;
fix15 d_error_fx = 0;
fix15 d_error_fy = 0;
fix15 dt = float2fix15(0.02);
fix15 control_smooth_alpha = float2fix15(.4);

fix15 integral_sum_x = 0;
fix15 integral_sum_y = 0;

fix15 i_error_fx = 0;
fix15 i_error_fy = 0;

fix15 p_error_fx = 0;
fix15 p_error_fy = 0;

fix15 prev_error_fx = 0;
fix15 prev_error_fy = 0;

fix15 p_term = float2fix15(0.32);
fix15 i_term = float2fix15(0.2);
fix15 d_term = float2fix15(4.8);
float integral_max = 40.0;

volatile int integral_sum = 0;
static int PLANE_CENTER_X = 150;
static int PLANE_CENTER_Y = 150;

static int CENTER_X = 178; // ACTUAL CENTER VALUES
static int CENTER_Y = 121;

static int NEW_CENTER_X = 178;
static int NEW_CENTER_Y = 121;

static int sqr_size = 200;

// GPIO we're using for PWM
#define LED_PIN 25
#define BUTTON_PIN 27

int prev_button_state = 0;
int curr_button_state = 0;
int button_state = 0;
int last_Button_state = 0;

////////////////////////////////////////////PWM code///////////////////////////////////

// Function to convert angle to PWM duty cycle level
uint16_t angle_to_level(int angle)
{

  angle = max(min(angle, MAX_ANGLE), MIN_ANGLE);
  int pulse_width_us = MIN_PULSE_US + (angle * (MAX_PULSE_US - MIN_PULSE_US)) / 270;
  return (pulse_width_us * WRAPVAL) / PERIOD_US; // scale to 20ms (50Hz)
}

// Function to change the current platform center
void change_center(int center_x, int center_y, float t)
{
  int r = 40;
  float phase = t * .03;
  int x_offset = (int)(r * (cos(phase) > 0 ? 1 : -1));
  int y_offset = (int)(r * (sin(phase) > 0 ? 1 : -1));
  NEW_CENTER_X = CENTER_X + x_offset;
  NEW_CENTER_Y = CENTER_Y + y_offset;
};

// Interrupt service routine
void on_pwm_wrap()
{
  // Clear the interrupt flag that brought us here
  pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT_1));

  // error from the camera
  error_fx = float2fix15(x_error);
  error_fy = float2fix15(y_error);

  // integral term
  integral_sum_x = integral_sum_x + multfix15(error_fx, dt);
  integral_sum_y = integral_sum_y + multfix15(error_fy, dt);
  if (integral_sum_x > float2fix15(integral_max))
  {
    integral_sum_x = float2fix15(integral_max);
  };
  if (integral_sum_x < float2fix15(-integral_max))
  {
    integral_sum_x = float2fix15(-integral_max);
  };
  if (integral_sum_y > float2fix15(integral_max))
  {
    integral_sum_y = float2fix15(integral_max);
  };
  if (integral_sum_y < float2fix15(-integral_max))
  {
    integral_sum_y = float2fix15(-integral_max);
  };

  i_error_fx = multfix15(integral_sum_x, i_term);
  i_error_fy = multfix15(integral_sum_y, i_term);

  // derivative term
  if (new_error)
  {
    d_error_fx = multfix15(error_fx - prev_error_fx, d_term);
    d_error_fy = multfix15(error_fy - prev_error_fy, d_term);
    if (abs(d_error_fx) > int2fix15(80))
      d_error_fx = 0;
    if (abs(d_error_fy) > int2fix15(80))
      d_error_fy = 0;
    prev_error_fx = error_fx;
    prev_error_fy = error_fy;
    new_error = 0;
  }

  // proportional term
  p_error_fx = multfix15(p_term, error_fx);
  p_error_fy = multfix15(p_term, error_fy);

  // add together and smooth
  control1 = multfix15(control_smooth_alpha, control1) + multfix15(int2fix15(1) - control_smooth_alpha, p_error_fx + d_error_fx + i_error_fx);
  control2 = multfix15(control_smooth_alpha, control2) + multfix15(int2fix15(1) - control_smooth_alpha, p_error_fy + d_error_fy + i_error_fy);

  // PWM to servo
  if (control1 != old_control1)
  {
    old_control1 = control1;
    pwm_set_chan_level(slice_num, PWM_CHAN_B, angle_to_level(-fix2int15(control1) + 135));
  }
  if (control2 != old_control2)
  {
    old_control2 = control2;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, angle_to_level(fix2int15(control2) + 135));
  }
  // Signal VGA to draw
  PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// for button control
static PT_THREAD(protothread_button(struct pt *pt))
{
  PT_BEGIN(pt);
  while (1)
  {
    int button_press = gpio_get(BUTTON_PIN);
    if (button_press && !prev_button_state)
    {
      button_state = !button_state;
    }
    prev_button_state = button_press;

    PT_YIELD_usec(100);
  }
  PT_END(pt);
}

////////////////////////////////////////////////////VGA CODE/////////////////////////////

// Draw the boundaries
void drawArena()
{
  fillCircle(PLANE_CENTER_X, PLANE_CENTER_Y, 2, GREEN);
  drawVLine(50, 50, sqr_size, WHITE);
  drawVLine(250, 50, sqr_size, WHITE);
  drawHLine(50, 50, sqr_size, WHITE);
  drawHLine(50, 250, sqr_size, WHITE);
}

// Draw the error lines
void drawErrorLines(int x, int y, char color)
{
  drawLine(PLANE_CENTER_X, PLANE_CENTER_Y, x, y, color);
  drawLine(x, y, PLANE_CENTER_X, y, color);
  drawLine(x, y, x, PLANE_CENTER_Y, color);
  drawLine(PLANE_CENTER_X, PLANE_CENTER_Y, PLANE_CENTER_X, y, color);
  drawLine(PLANE_CENTER_X, PLANE_CENTER_Y, x, PLANE_CENTER_Y, color);
}

// Thread that draws to VGA display
static PT_THREAD(protothread_vga(struct pt *pt))
{
  // Indicate start of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;
  static int prevball_x = -1, prevball_y = -1;

  // We will start drawing at column 81
  static int xcoord = 81;
  static int plt_start = 230;
  // Control rate of drawing
  static int throttle;
  static float NewRange = 150.; // (looks nice on VGA)

  static int counter = 0;

  int currball_x, currball_y;
  setTextSize(1);
  setTextColor(WHITE);

  // Draw bottom plot
  sprintf(screentext, "PID control");
  setCursor(320 + 150, 260);
  drawHLine(75 + plt_start, 430, 5, CYAN);
  drawHLine(75 + plt_start, 355, 5, CYAN);
  drawHLine(75 + plt_start, 280, 5, CYAN);
  drawVLine(80 + plt_start, 280, 150, CYAN);
  sprintf(screentext, "0");
  setCursor(50 + plt_start, 350);
  writeString(screentext);
  sprintf(screentext, "80");
  setCursor(50 + plt_start, 280);
  writeString(screentext);
  sprintf(screentext, "-80");
  setCursor(50 + plt_start, 425);
  writeString(screentext);

  // Draw top plot
  sprintf(screentext, "servo angle");
  setCursor(320, 40);
  drawHLine(75 + plt_start, 230, 5, CYAN);
  drawHLine(75 + plt_start, 155, 5, CYAN);
  drawHLine(75 + plt_start, 80, 5, CYAN);
  drawVLine(80 + plt_start, 80, 150, CYAN);
  sprintf(screentext, "0");
  setCursor(50 + plt_start, 150);
  writeString(screentext);
  sprintf(screentext, "80");
  setCursor(45 + plt_start, 75);
  writeString(screentext);
  sprintf(screentext, "-80");
  setCursor(45 + plt_start, 225);
  writeString(screentext);

  static int target_x = -1;
  static int target_y = -1;

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();

    currball_x = 150 + ((int)(x_error + NEW_CENTER_X - CENTER_X) * (sqr_size / 2)) / 80;
    currball_y = 150 + ((int)(y_error + NEW_CENTER_Y - CENTER_Y) * (sqr_size / 2)) / 80;
    if (prevball_x >= 0)
    {
      drawCircle(prevball_x, prevball_y, 10, BLACK);
      drawErrorLines(prevball_x, prevball_y, BLACK);
    }
    prevball_x = currball_x;
    prevball_y = currball_y;
    drawArena();
    drawCircle(currball_x, currball_y, 10, GREEN);

    if (button_state)
    {
      p_term = float2fix15(0.2);
      i_term = float2fix15(.5);
      d_term = float2fix15(4);
      integral_max = 10.0;
      control_smooth_alpha = float2fix15(.8);
      change_center(0, 0, counter);
      counter++;
    }
    else
    {
      p_term = float2fix15(0.32);
      i_term = float2fix15(0.2);
      d_term = float2fix15(4.8);
      control_smooth_alpha = float2fix15(.3);
      NEW_CENTER_X = CENTER_X;
      NEW_CENTER_Y = CENTER_Y;
      integral_max = 40.0;
    }
    if (target_x > 0)
    {
      fillCircle(target_x, target_y, 2, BLACK);
    }
    target_x = 150 + ((int)(NEW_CENTER_X - CENTER_X) * (sqr_size / 2)) / 80;
    target_y = 150 + ((int)(NEW_CENTER_Y - CENTER_Y) * (sqr_size / 2)) / 80;
    fillCircle(target_x, target_y, 2, BLUE);

    drawErrorLines(currball_x, currball_y, WHITE);

    // Increment drawspeed controller
    throttle += 1;
    // If the controller has exceeded a threshold, draw
    if (throttle >= threshold)
    {
      // Zero drawspeed controller
      throttle = 0;
      drawVLine(xcoord + plt_start, 0, 480, BLACK);

      // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
      drawPixel(xcoord + plt_start, 430 - NewRange / 2 - (int)(NewRange * ((float)(MAX_ANGLE - 135) / 160.0)), WHITE);
      drawPixel(xcoord + plt_start, 430 - NewRange / 2 - (int)(NewRange * ((float)(fix2float15(d_error_fx)) / 160.0)), BLUE);
      drawPixel(xcoord + plt_start, 430 - NewRange / 2 - (int)(NewRange * ((float)(fix2float15(p_error_fx)) / 160.0)), GREEN);
      drawPixel(xcoord + plt_start, 430 - NewRange / 2 - (int)(NewRange * ((float)(fix2int15(control1)) / 160.0)), WHITE);
      drawPixel(xcoord + plt_start, 430 - NewRange / 2 - (int)(NewRange * ((float)(MIN_ANGLE - 135) / 160.0)), WHITE);
      // Draw top plot
      drawPixel(xcoord + plt_start, 230 - NewRange / 2 - (int)(NewRange * ((float)(MAX_ANGLE - 135) / 160.0)), WHITE);
      drawPixel(xcoord + plt_start, 230 - NewRange / 2 - (int)(NewRange * ((float)(fix2int15(control2)) / 160.0)), WHITE);
      drawPixel(xcoord + plt_start, 230 - NewRange / 2 - (int)(NewRange * ((float)(MIN_ANGLE - 135) / 160.0)), WHITE);

      // Update horizontal cursor
      if (xcoord < 400)
      {
        xcoord += 1;
      }
      else
      {
        xcoord = 81;
      }
    }

    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)

  // Indicate end of thread
  PT_END(pt);
}

/////////////////////////////////////////////////communication/////////////////////////////////////
// Thread for uart communication with the pi
static PT_THREAD(protothread_uart(struct pt *pt))
{
  // Indicate start of thread
  PT_BEGIN(pt);
  int num1, num2;
  char buffer[ubuf_size];
  int idx = 0;
  bool receiving = false;
  static int buffer_pointer = 0;
  static float smooth = 0.4; // 0 is no smooth .99 is max smooth
  static float smoothed_x = 0.0;
  static float smoothed_y = 0.0;

  while (1)
  {
    while (uart_is_readable(UART_ID1))
    {
      char ch = uart_getc(UART_ID1);
      if (!receiving)
      {
        // wait for start of tuple
        if (ch == '(')
        {
          receiving = true;
          idx = 0;
          buffer[idx++] = ch;
        }
      }
      else
      {
        buffer[idx++] = ch;
        // stop for overflow
        if (idx >= ubuf_size - 1)
        {
          receiving = false;
          idx = 0;
          continue;
        }

        if (ch == ')')
        {
          // complete message
          buffer[idx] = '\0';
          receiving = false;

          if (sscanf(buffer, "(%d,%d)", &num1, &num2) == 2)
          {
            smoothed_x = smooth * smoothed_x + (1 - smooth) * num1;
            smoothed_y = smooth * smoothed_y + (1 - smooth) * num2;
            x_error = max(min(smoothed_x - (float)NEW_CENTER_X, 80.0), -80.0);
            y_error = max(min(smoothed_y - (float)NEW_CENTER_Y, 80.0), -80.0);
            new_error = 1;
          }
          idx = 0;
        }
      }
    }
  } // END WHILE(1)

  // Indicate end of thread
  PT_END(pt);
}

//////////////////////////////////////////////////////////////main threads//////////////////////////////
// Entry point for core 1
void core1_entry()
{
  pt_add_thread(protothread_vga);
  pt_add_thread(protothread_button);
  pt_schedule_start;
}

int main()
{
  // Initialize stdio
  stdio_init_all();

  // Initialize VGA
  initVGA();

  gpio_init(LED_PIN);
  // Configure the LED pin as an output
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);

  // setup UART
  uart_init(UART_ID1, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  printf("UART Ready");

  ////////////////////////////////////////////////////////////////////////
  ///////////////////////// PWM CONFIGURATION ////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  // Setup PWM on GPIO
  gpio_set_function(PWM_OUT_1, GPIO_FUNC_PWM);
  gpio_set_function(PWM_OUT_2, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(PWM_OUT_1);

  // Configure PWM frequency
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, CLKDIV);
  pwm_config_set_wrap(&config, WRAPVAL);
  pwm_init(slice_num, &config, true);

  pwm_set_output_polarity(slice_num, 1, 1);

  // Start at neutral position (90°)
  pwm_set_chan_level(slice_num, PWM_CHAN_A, angle_to_level(135));
  pwm_set_chan_level(slice_num, PWM_CHAN_B, angle_to_level(135));

  // Enable interrupt
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // Start PWM output
  pwm_set_mask_enabled((1u << slice_num));

  ////////////////////////////////////////////////////////////////////////
  ///////////////////////////// ROCK AND ROLL ////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // start core 0
  pt_add_thread(protothread_uart);
  pt_schedule_start;
}

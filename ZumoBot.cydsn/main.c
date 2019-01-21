#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
/**
 * @file    main.c
 * @brief  
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/
 
void motor_arbit(int l_speed, int r_speed, uint32 delay);
void button_wait(int delay);
 
// Project
// Line follower
#if 1
void cross_line(struct sensors_, int max_speed);
void zmain(void) {
    struct sensors_ dig;
   
    // you may change these
    int const max_speed = 160;      // maximum speed, 0...255
    int const med_speed = max_speed / 2;
    int const low_speed = max_speed / 5;
    int const min_speed = -20;
    int const rate_s = 1;           // slow rate of change
    int const rate_f = rate_s * 4;  // fast rate
    int const delay = 0;
   
    // reflectance threshold; default: center sensors 11000, others 9000
    int const th1 = 14000;
    int const th2 = 9000;
    int const th3 = 9000;
   
    int left_speed = 0, right_speed = 0, time = 0, ontrack = 1;  // ontrack = 1 when on track
    TickType_t start = 0, stop = 0;
   
    reflectance_start();
    reflectance_set_threshold(th3, th2, th1, th1, th2, th3);
   
    motor_start();
    motor_forward(0, 0);
   
    button_wait(750);
    motor_forward(50, 0);
   
    for (int i = 0; i < 3; ++i) {
        reflectance_digital(&dig);
       
        // start and finish line
        if (i == 0) {                      
            // drive forward unil on start line
            while (!dig.l3 && !dig.l2 && !dig.r2 && !dig.r3) {
                reflectance_digital(&dig);
                vTaskDelay(1);
            }
            vTaskDelay(20);
            // stop at start line
            motor_forward(0, 0);
            IR_Start();
            IR_flush();
            print_mqtt("Zumo010/ready", "line");
            // wait for IR command
            IR_wait();
            start = xTaskGetTickCount();
            print_mqtt("Zumo010/start", "%d", start);
            left_speed = right_speed = max_speed;
            cross_line(dig, max_speed);
        } else if (i == 1) {    // ignore first line
            cross_line(dig, max_speed);
        } else {                // stop at finish line
            motor_forward(0, 0);
            stop = xTaskGetTickCount();
            print_mqtt("Zumo010/stop", "%d", stop);
            time = stop - start;
            print_mqtt("Zumo010/time", "%d", time);
            break;
        }
        for (;;) {
            reflectance_digital(&dig);
            if (dig.l3 && dig.l2 && dig.r2 && dig.r3) {
                break;
            }
           
           
            // left side control
            if (dig.l3) {        // hard turn
                left_speed = min_speed;
            } else if (dig.l2) { // normal turn
                left_speed -= rate_f;
                if (left_speed < low_speed) {
                    left_speed = low_speed;
                }
            } else if (dig.l1) {
                if (dig.r1) {               // full speed
                    left_speed = max_speed;
                } else {                    // soft turn
                    left_speed -= rate_s;
                    if (left_speed < med_speed) {
                        left_speed = med_speed;
                    }
                }
            }
           
            // right side control
            if (dig.r3) {         // hard turn
                right_speed = min_speed;
            } else if (dig.r2) {  // normal turn
                right_speed -= rate_f;
                if (right_speed < low_speed) {
                    right_speed = low_speed;
                }
            } else if (dig.r1) {
                if (dig.l1) {                   // full speed
                    right_speed = max_speed;
                } else {                        // soft turn
                    right_speed -= rate_s;
                    if (right_speed < med_speed) {
                        right_speed = med_speed;
                    }
                }
            }
           
            // bonus
            if (ontrack == 1 && !dig.l1 && !dig.r1) {
                ontrack = 0;
                print_mqtt("Zumo010/miss", "%d", xTaskGetTickCount());
            } else if (ontrack == 0 && dig.l1 && dig.r1) {
                ontrack = 1;
                print_mqtt("Zumo010/line", "%d", xTaskGetTickCount());
            }
           
            // set motor speeds
            motor_arbit(left_speed, right_speed, delay);
        }
    }
}
// keep driving at given speed until both side sensors see white
void cross_line(struct sensors_ dig, int speed) {
    motor_forward(speed, 0);
    while (dig.l3 || dig.r3) {
        reflectance_digital(&dig);
        vTaskDelay(1);
    }
    vTaskDelay(10);
}
#endif
 
// Zumo sumo
#if 0
// mood integer names for readability
#define STROLL 0
#define FIND_CENTER 1
#define PICK_A_FIGHT 2
#define ATTACK 3
 
int middle_of_3(int a, int b, int c);
   
void zmain(void)
{
    struct sensors_ dig;
    struct accData_ data;
   
    int const stroll_speed = 200, reverse_speed = -200, attack_speed = 255, hit_threshold = 16200;
   
    int left = 0, right = 0;
    int counter = 0, timer = 0, accel = 0;
    int coinflip, distance, mood;
   
    int x_acc[3] = {0, 0, 0}, x_val = 0;
    int y_acc[3] = {0, 0, 0}, y_val = 0;
    int hit_force = 0, hit_angle_int = 0;
    double hit_angle = 0;
   
    TickType_t start_time = 0, stop_time = 0, current_time = 0;
   
    reflectance_start();
    Ultra_Start();
    LSM303D_Start();
    motor_start();
    motor_arbit(0, 0, 0);
    IR_Start();
    IR_flush();
   
    reflectance_set_threshold(12000, 12000, 14000, 14000, 12000, 12000);
   
    // drive to the edge and stop
    button_wait(750);
    motor_arbit(50, 50, 0);
    do {
        reflectance_digital(&dig);
        vTaskDelay(1);
    } while (!dig.l3 && !dig.r3);
    vTaskDelay(20);
    motor_arbit(0, 0, 0);
   
    // wait for start command
    print_mqtt("Zumo010/ready", "zumo");
    IR_wait();
   
    // tachi-ai!
    start_time = xTaskGetTickCount();
    print_mqtt("Zumo010/start", "%d", start_time);
    motor_forward(stroll_speed, 100);
    IR_flush();
   
    // turn randomly to left or right at the tachi-ai
    coinflip = rand() % 2;
    if (coinflip == 0) {
        motor_arbit(stroll_speed / 4, stroll_speed, 250);
    } else {
        motor_arbit(stroll_speed, stroll_speed / 4, 250);
    }
    mood = STROLL;
   
    for (;;) {
        // update sensor data
        reflectance_digital(&dig);
        LSM303D_Read_Acc(&data);
        distance = Ultra_GetDistance();
       
        // act according to mood
        switch (mood) {
            // drive forward until an edge or an opponent is detected
            case STROLL:
                left = right = stroll_speed;
                break;
           
            // try to go close to the center to look for opponents
            case FIND_CENTER:
                if (++timer > 500) {
                    mood = PICK_A_FIGHT;
                    timer = 0;
                    coinflip = rand() % 2;     // flip a coin
                }
                break;
           
            // run around in a circle to find opponents
            case PICK_A_FIGHT:
                // select direction of circle according to coin flip
                if (coinflip == 0) {
                    left = 0;
                    right = stroll_speed;
                } else {
                    left = stroll_speed;
                    right = 0;
                }
                // creep towards distant opponent
                if (distance < 30) {
                    left = right = attack_speed / 2;
                }
                // stop looking for opponents after a while
                if (++timer > 2500) {
                    mood = STROLL;
                }
                break;
               
            // charge towards an opponent
            case ATTACK:
                left = right = attack_speed / 2;
                // accelerate gradually to keep reflectance sensors working as expected
                left += accel * 2;
                right += accel * 2;
                if (left > attack_speed) {
                    left = attack_speed;
                }
                if (right > attack_speed) {
                    right = attack_speed;
                }
                ++accel;
                // stop attacking if opponent gets away
                if (distance > 20) {
                    mood = STROLL;
                }
                break;
        }
       
        // turn towards the center if an edge is detected on both sides
        if ((dig.l3 || dig.l2) && (dig.r2 || dig.r3)) {
            motor_arbit(reverse_speed, reverse_speed, 50);
            motor_arbit(reverse_speed, stroll_speed, 400);
            mood = FIND_CENTER;
            timer = 0;
        } else if (dig.l3 || dig.l2 || dig.l1) {    // reverse with the side where an edge is detected, if not on both sides
            left = reverse_speed / 3;
            mood = STROLL;
        } else if (dig.r3 || dig.r2 || dig.r1) {
            right = reverse_speed / 3;
            mood = STROLL;
        } else if (distance < 18 && mood != ATTACK) {   // check for nearby opponents if no edge is detected
            mood = ATTACK;
            accel = 0;
        }
       
        // hit detector
        x_acc[counter % 3] = data.accX;
        y_acc[counter % 3] = data.accY;
        // every three iterations, add the median of the last three values
        if (counter % 3 == 2) {
            x_val += middle_of_3(x_acc[0], x_acc[1], x_acc[2]);
            y_val += middle_of_3(y_acc[0], y_acc[1], y_acc[2]);
        }
        // every nine iterations, take the average of the last three medians
        // and check to see if we've been hit
        if (counter % 6 == 5) {
            x_val /= 2;
            y_val /= 2;
            hit_force = sqrt((x_val * x_val) + (y_val * y_val));
            if (hit_force > hit_threshold) {
                // flip y-axis, calculate hit angle, convert to degrees and send data to MQTT broker
                hit_angle = atan2(y_val * -1, x_val) * 180 / M_PI + 180;
                hit_angle_int = hit_angle;
                current_time = xTaskGetTickCount();
                print_mqtt("Zumo010/hit", "%d %d", current_time, hit_angle_int % 360);
            }
            // reset
            x_val = y_val = 0;
        }
        ++counter;
       
        // stop doing sumo when button is pressed
        if (SW1_Read() == 0) {
            while (SW1_Read() == 0) vTaskDelay(10);
            motor_forward(0, 0);
            stop_time = xTaskGetTickCount();
            print_mqtt("Zumo010/stop", "%d", stop_time);
            print_mqtt("Zumo010/time", "%d", stop_time - start_time);
            break;
        }
       
        // set motor speeds and wait 1 ms before next iteration
        motor_arbit(left, right, 1);
    }
 }
int middle_of_3(int a, int b, int c) {
    int middle;
   
    if ((a <= b) && (a <= c)) {
        middle = (b <= c) ? b : c;
    } else if ((b <= a) && (b <= c)) {
        middle = (a <= c) ? a : c;
    } else {
        middle = (a <= b) ? a : b;
    }
    return middle;
}
#endif
 
// Maze
#if 0
//name integers for easier readability
#define FORWARD 0
#define RIGHT 1
#define LEFT 2
 
void turn_left(struct sensors_ dig, int turn_speed, int turn_delay);
void turn_right(struct sensors_ dig, int turn_speed, int turn_delay);
 
void zmain(void)
{
   
    int x = 0, y = -1;
    /*coordinates. y is along to long axis that goes from start to finish
    x is perpendicular, shorter axis- y starts at -1 to offset starting before first  intersection*/
    int counter = 0; //counter so that robot stops after going without intersection for a while
    int d = 50; //d is distance seen by ultrasound
    int orientation = FORWARD; //direction robot is facing, always starts by going FORWARD
    struct sensors_ dig;
   
    TickType_t start = 0, stop = 0; //start and stop time
   
    Ultra_Start();
    motor_start();
    IR_Start();
    IR_flush();
    reflectance_start();
   
    // you may change these
    int const max_speed = 160;    // maximum speed, 0...255
    int const turn_speed = 110;   // maximum turnspeed 0..255
    int const centering_delay = 95; //how long robot goes straight at an intersection to center itself
    int const turn_delay = 200;   // how long robot turns before starting to check bottom sensors again
    int const rate_s = 1;         // slow rate of change
    int const max_intersectionless = 450;     //how long robot can go without sensing an intersection
    int leftmotor = max_speed, rightmotor = max_speed; //starting values for left and right motor. Should be lower than max_speed
   
    reflectance_set_threshold(12000, 12000, 17000, 17000, 12000, 12000);
   
    print_mqtt("Zumo010/ready", "maze");
    button_wait(750);
    motor_arbit(50, 50, 0);
   
    do {    // drive to first line
        reflectance_digital(&dig);
        vTaskDelay(5);
    } while (!dig.l3 && !dig.r3);
    //stops and waits for go signal from IR remote
    motor_arbit(0, 0, 0);
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo010/start", "%d", start);
    motor_arbit(max_speed, max_speed, 100); //drives over the starting line
    for(;;) {
        reflectance_digital(&dig);
        if ((dig.l3 && dig.l2 && dig.l1) || (dig.r1 && dig.r2 && dig.r3)) { //check for intersection
            motor_arbit(max_speed, max_speed, centering_delay); //drive forward a little so center point of robot closer to intersection
            counter = 0; //reset counter at intersection
            //change coordinates
            if (orientation == FORWARD) {
                y++;
            } else if (orientation == LEFT) {
                x--;
            } else if (orientation == RIGHT) {
                x++;
            }
            if (y >= 11) { //at the end of maze on the last full row, robot goes to the middle of x axis and goes straight to goal
                if (orientation == FORWARD) {
                    if (x < 0) {
                        turn_right(dig, turn_speed, turn_delay);
                        orientation = RIGHT;
                    } else if (x > 0) {
                        turn_left(dig, turn_speed, turn_delay);
                        orientation = LEFT;
                    }
                } else if (orientation == RIGHT) {
                    if (x == 0) {
                        turn_left(dig, turn_speed, turn_delay);
                        orientation = FORWARD;
                    }
                } else if (orientation == LEFT) {
                    if (x == 0) {
                        turn_right(dig, turn_speed, turn_delay);
                        orientation = FORWARD;
                    }
                }
            //going trough the maze normally    
            } else if (orientation == FORWARD) { //while going forward, only stops and turns at intersection if ultrasound detects something blocking the path
                d = Ultra_GetDistance();
                if (d < 20) {
                    if (x <= 0) { //if on the left side or middle, tries turning to right first
                        turn_right(dig, turn_speed, turn_delay);
                        orientation = RIGHT;
                        d = Ultra_GetDistance();
                        if (d < 20) {
                            turn_left(dig, turn_speed, turn_delay);
                            turn_left(dig, turn_speed, turn_delay);
                            orientation = LEFT;
                        }
                    } else if (x > 0) { //if on the right side, tries turning to left first
                        turn_left(dig, turn_speed, turn_delay);
                        orientation = LEFT;
                        d = Ultra_GetDistance();
                        if (d < 20) {
                            turn_right(dig, turn_speed, turn_delay);
                            turn_right(dig, turn_speed, turn_delay);
                            orientation = RIGHT;
                        }
                    }
                } else {
                    motor_arbit(max_speed, max_speed, 100);
                }
            } else if (orientation == LEFT) {
                /*while going left, tries turning towards the goal every intersection to check if anything is blocking the way
                otherwise, goes to the left until it reaches the end of of x axis or something blocks the way. If going forward is still not possible, starts going
                right*/
                d = Ultra_GetDistance();
                if (d < 20 || x == -3) { //checks if anything is blocking the way or if at the end of x axis
                    turn_right(dig, turn_speed, turn_delay);
                    orientation = FORWARD;
                    d = Ultra_GetDistance();
                    if (d < 20) {
                        turn_right(dig, turn_speed, turn_delay);
                        orientation = RIGHT;
                    }
                } else { //if nothing blocking, check if forward possible, if not, keeps going left
                    turn_right(dig, turn_speed, turn_delay);
                    orientation = FORWARD;
                    d = Ultra_GetDistance();
                    if (d < 20) {
                        turn_left(dig, turn_speed, turn_delay);
                        orientation = LEFT;
                    }
                }
            } else if (orientation == RIGHT) {
                 /*while going right, tries turning towards the goal every intersection to check if anything is blocking the way
                otherwise, goes to the rigght until it reaches the end of of x axis or something blocks the way. If going forward is still not possible, starts going
                left*/
                d = Ultra_GetDistance();
                if (d < 20 || x == 3) { //checks if anything is blocking the way or if at the end of x axis
                    turn_left(dig, turn_speed, turn_delay);
                    orientation = FORWARD;
                    d = Ultra_GetDistance();
                    if (d < 20) {
                        turn_left(dig, turn_speed, turn_delay);
                        orientation = LEFT;
                    }
                } else { //if nothing blocking, check if forward is possible, if not, keeps going right
                    turn_left(dig, turn_speed, turn_delay);
                    orientation = FORWARD;
                    d = Ultra_GetDistance();
                    if (d < 20) {
                        turn_right(dig, turn_speed, turn_delay);
                        orientation = RIGHT;
                    }
                }
            }
            print_mqtt("Zumo010/position", "%d %d", x, y);
        }
        //Driving along (straight) line with minor adjustations
        if (dig.l1) {
            if (dig.r1) {           // full speed
                leftmotor = max_speed;
            } else if (leftmotor > max_speed/2) {  // soft turn
                leftmotor -= rate_s;
            }
        }
        if (dig.r1) {
            if (dig.l1) {           // full speed
                rightmotor = max_speed;
            } else if (rightmotor > max_speed/2) {  // soft turn
                rightmotor -= rate_s;
            }
        }
        counter++; //adds to counter
       
        if (counter >= max_intersectionless && y >= 13) { //if counter reaches specific value, robot has been driving straight without an intersection and stops
            motor_arbit(0, 0, 0);
            break;
        }
       
        motor_arbit(leftmotor, rightmotor, 1);
    }
    //get the stop time and calculate time
    stop = xTaskGetTickCount();
    print_mqtt("Zumo010/stop", "%d", stop);
    print_mqtt("Zumo010/time", "%d", stop-start);
   
}  
//vasemmalle kääntyminen risteyksessä, kunnes sensorit huomaavat mustan viivan
void turn_left(struct sensors_ dig, int turn_speed, int turn_delay) {
    motor_arbit(-turn_speed, turn_speed, turn_delay);
    for (;;) {
        motor_arbit(-turn_speed, turn_speed, 0);
        reflectance_digital(&dig);
        if (dig.r1) {
            motor_arbit(0, 0, 0);
            break;
        }
    }
}
//oikealle kääntyminen risteyksessä, kunnes sensorit huomaavat mustan viivan
void turn_right(struct sensors_ dig, int turn_speed, int turn_delay) {
    motor_arbit(turn_speed, -turn_speed, turn_delay);
    for (;;) {
        motor_arbit(turn_speed, -turn_speed, 0);
        reflectance_digital(&dig);
        if (dig.l1) {
            motor_arbit(0, 0, 0);
            break;
        }
    }
}
#endif
 
// Motor function that can take negative values for reversing
void motor_arbit(int l_speed, int r_speed, uint32 delay)
{
    if (l_speed < 0) {
        MotorDirLeft_Write(1);
        PWM_WriteCompare1(l_speed * -1);
    } else {
        MotorDirLeft_Write(0);
        PWM_WriteCompare1(l_speed);
    }
    if (r_speed < 0) {
        MotorDirRight_Write(1);
        PWM_WriteCompare2(r_speed * -1);
    } else {
        MotorDirRight_Write(0);
        PWM_WriteCompare2(r_speed);
    }
    vTaskDelay(delay);
}
// wait until button is pressed, then pause for given time
void button_wait(int delay) {
    for (;;) {
        if (SW1_Read() == 0) {
            while (SW1_Read() == 0) vTaskDelay(10);
            break;
        }
        vTaskDelay(10);
    }
    vTaskDelay(delay);
}
/* [] END OF FILE */
/*
 * ui.h
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#ifndef INC_UI_H_
#define INC_UI_H_

volatile extern int8_t ui_tid;
#define UI_EV_1S (1)
#define UI_EV_TIME_UPDATE (2)
#define UI_EV_BUTTON0 (3)
#define UI_EV_BUTTON1 (4)
#define UI_EV_BUTTON2 (5)
#define UI_EV_BUTTON3 (6)
#define UI_EV_BUTTON4 (7)

void ui_init(void);

#endif /* INC_UI_H_ */

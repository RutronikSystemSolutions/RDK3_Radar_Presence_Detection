/*
 * rutronik_app.h
 *
 *  Created on: 20 Jul 2023
 *      Author: jorda
 */

#ifndef RUTRONIK_APP_H_
#define RUTRONIK_APP_H_

/**
 * @brief Initializes the application logic
 *
 * @retval 0 Success
 * @retval !0 Something wrong happened
 */
void rutronik_app_init();

/**
 * @brief Cyclic call to the app
 */
void rutronik_app_do();


#endif /* RUTRONIK_APP_H_ */

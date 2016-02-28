/*
 * menu.h
 *
 *  Created on: november 11 2015
 *      Author: patrick
 */

#ifndef BANNER_H_
#define BANNER_H_

enum iconType {LINESENSORS, TELEMETERS, GYROMETER, BEEPER, BLUETOOTH, BATTERY, USB};

void bannerSetIcon(enum iconType icon, int val);

#endif /* BANNER_H_ */

/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef OBJECT_AVOIDER_H
#define OBJECT_AVOIDER_H

// settings
extern float oa_color_count_frac;

// functions
extern void object_avoider_init(void);
extern void object_avoider_periodic(void);

#endif


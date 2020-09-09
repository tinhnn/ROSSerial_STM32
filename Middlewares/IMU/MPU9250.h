/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

class MPU9250
{
    private:
    public:
        MPU9250();

        float quat[4];
};

#endif /* MPU9250_H */

/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef DFR0592_H
#define DFR0592_H

class DFR0592
{
private:
    /* data */
public:
    DFR0592(/* args */);
    ~DFR0592();
    
    void init(void);
    bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);
    float* getOrientation(void);
};



#endif /* DFR0592_H */

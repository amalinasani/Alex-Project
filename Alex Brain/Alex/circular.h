#pragma once
#include <string.h>

class circular
{
private:
    unsigned char global_buffer[300]; //global buffer to store our stuff
    int front_pointer;
    int back_pointer;

public:
    circular();
    int write_buffer(void* info, int size); //give it the memory location of the information and it will copy there
    int read_buffer(void* storage, int size); //read the current buffer and store information there
    void buffer_clear();

};

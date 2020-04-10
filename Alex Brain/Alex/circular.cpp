#include "circular.h"


#define array_size 200
circular::circular()
{
    front_pointer = 0;
    back_pointer = 0;
}

void circular::buffer_clear() //clear the entire memory address by resetting the pointer, we do not erase
{
    front_pointer = 0;
    back_pointer = 0;
}

int circular::write_buffer(void* info, int size)
{
    unsigned char temp_buffer[array_size]; //create a temp buffer to hold stuff here first
    int max_allowed;

    //first, calculate how much leeway we have to implement the array
    if (front_pointer < back_pointer) //the front pointer is behind the back pointer, calculate how much space left
    {
        max_allowed = (back_pointer - front_pointer);

    }

    else
    {
        max_allowed = array_size - (front_pointer - back_pointer);
    }

    if ((size / sizeof(char)) >= max_allowed) //we cannot store stuff here, too little information
    {
        //cout << "Too much information" << endl;
        return 2;
    }

    else
    {
        memcpy(temp_buffer, info, size);
        for (int i = 0; i < (size / sizeof(char)); i++)
        {
            memcpy(&global_buffer[front_pointer], &temp_buffer[i], sizeof(char));
            front_pointer = (front_pointer == array_size - 1) ? 0 : front_pointer + 1;
        }
        return 0;


    }
}

int circular::read_buffer(void* storage, int size)
{
    int amount;
    int max_allowed;
    unsigned char temp_buffer[array_size];

    amount = size / sizeof(char);

    //first, calculate how much stuff there is to read from
    if (back_pointer <= front_pointer) //keep moving forward till reach front pointer
    {
        max_allowed = front_pointer - back_pointer;
    }
    else
    {
        max_allowed = array_size - (back_pointer - front_pointer);
    }

    if (amount > max_allowed)
    {
        //cout << "Too little information" << endl;
        return 1;
    }

    else
    {
        for (int i = 0; i < amount; i++)
        {
            memcpy(&temp_buffer[i], &global_buffer[back_pointer], sizeof(char));
            back_pointer = (back_pointer == array_size - 1) ? 0 : back_pointer + 1;
        }

        memcpy(storage, temp_buffer, size);
        return 0;
    }
}

#pragma once

////////////////////// NOT MY CODE //////////////////////////////////////////
// https://wickedengine.net/2018/11/24/simple-job-system-using-standard-c/ //
/////////////////////////////////////////////////////////////////////////////

// Fixed size very simple thread safe ring buffer
template <typename T, size_t capacity>
class RingBuffer
{
public:
    // Push an item to the end if there is free space
    //  Returns true if succesful
    //  Returns false if there is not enough space
    inline bool push_back(const T& item)
    {
        bool result = false;
        size_t next = (head + 1) % capacity;
        if (next != tail)
        {
            data[head] = item;
            head = next;
            result = true;
        }
        return result;
    }
 
    // Get an item if there are any
    //  Returns true if succesful
    //  Returns false if there are no items
    inline bool pop_front(T& item)
    {
        bool result = false;
        if (tail != head)
        {
            item = data[tail];
            tail = (tail + 1) % capacity;
            result = true;
        }
        return result;
    }
 
private:
    T data[capacity];
    size_t head = 0;
    size_t tail = 0;
};



#ifndef BUFFER_HPP
#define BUFFER_HPP

template <typename T, int SIZE>
class Buffer {
public:

    Buffer() :
    currentHead(0),
    currentTail(0),
    moveTail(false) {

    }

    void insert(T value) volatile {
        buffer[currentHead] = value;
        currentHead = (currentHead + 1) % SIZE;

        if(currentHead - currentTail >= SIZE)
            moveTail = true;

        if(moveTail)
            currentTail = (currentTail + 1) % SIZE;
    }

    T getBegin(int index) volatile {
        return buffer[(currentHead - index) % SIZE];
    }

    T getEnd(int offset) volatile {
        return buffer[(currentTail + offset) % SIZE];
    }

    int getCurrentHead() volatile {
        return currentHead;
    }

    int getCurrentTail() volatile {
        return currentTail;
    }

    volatile T* _getBufferLocation() volatile {
        return buffer;
    }

private:

    int currentHead, currentTail;
    bool moveTail;
    T buffer[SIZE];
};

#endif // BUFFER_HPP
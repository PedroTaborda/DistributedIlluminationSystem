#ifndef BUFFER_HPP
#define BUFFER_HPP

template <typename T, int SIZE>
// indexing starts at 1
class Buffer {
   public:
    Buffer() { reset(); }

    void reset() volatile {
        items = 0;
        currentHead = 0;
    }

    void insert(T value) volatile {
        buffer[currentHead] = value;
        currentHead = (currentHead + 1) % SIZE;
        items = min(SIZE, items + 1);
    }

    T getBegin(int index) volatile { return buffer[(currentHead - index) % SIZE]; }

    T indexFromCustomHead(int index, int customHead) volatile { return buffer[(customHead + index) % SIZE]; }

    // number of available items
    int available() volatile { return items; }

    // T getEnd(int offset) volatile { return buffer[(currentHead - items + offset - 1) % SIZE]; }

    int getCurrentHead() volatile { return currentHead; }

    volatile T* _getBufferLocation() volatile { return buffer; }

   private:
    int currentHead, items;
    T buffer[SIZE];
};

#endif  // BUFFER_HPP
template <class T, int SIZE> class CircularBuffer {
public:
    CircularBuffer();
    ~CircularBuffer();
    void push(T item);
    T pop();
    T get(int index); // 0 is the most recent item
    T getRaw(int index);
    void setRaw(int index, T item) { buffer[index] = item; }
    bool full() { return num_items == SIZE; }
    int length() { return num_items; }
private:
    T buffer[SIZE];
    int next_index;
    int num_items;
};

int mod(int k, int n) {
    return ((k %= n) < 0) ? k+n : k;
}

template <class T, int SIZE> CircularBuffer<T, SIZE>::CircularBuffer() {
    next_index = 0;
    num_items = 0;
}

template <class T, int SIZE> CircularBuffer<T, SIZE>::~CircularBuffer() {
}

template <class T, int SIZE> void CircularBuffer<T, SIZE>::push(T item) {
    buffer[next_index] = item;
    next_index = (next_index + 1) % SIZE;
    if (num_items < SIZE) {
        num_items++;
    }
}

// Returns the oldest item and removes it from the buffer
template <class T, int SIZE> T CircularBuffer<T, SIZE>::pop() { // FIFO
    if (num_items == 0) {
        return T();
    }

    int i = mod(next_index - num_items, SIZE);

    num_items--;

    // Serial.print("POP: ");
    // Serial.print(i);
    // Serial.print(", ");
    // Serial.print(next_index);
    // Serial.print(" is: '");
    // Serial.print(buffer[i]);
    // Serial.println("'");

    return buffer[i];
}


// 0 is the most recent item
template <class T, int SIZE> T CircularBuffer<T, SIZE>::get(int index) {
    // index of most recent item is next_index - 1
    // index of oldest item is next_index
    int i = mod(next_index - index - 1, SIZE);

    // Serial.print("INDEX: ");
    // Serial.print(index);
    // Serial.print(", ");
    // Serial.println(i);



    return buffer[i];
}

template <class T, int SIZE> T CircularBuffer<T, SIZE>::getRaw(int index) {
    return buffer[index];
}
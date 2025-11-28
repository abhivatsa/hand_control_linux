#include <limits>
#include "merai/RTIpc.h"

int main()
{
    using namespace merai;

    // DoubleBuffer publish/read path
    DoubleBuffer<int> buf{};
    int back = back_index(buf);
    buf.buffer[back] = 42;
    publish(buf, back);

    int value = 0;
    read_snapshot(buf, value);
    if (value != 42)
    {
        return 1;
    }

    // Servo Rx/TX helpers
    DoubleBuffer<std::array<ServoRxPdo, MAX_SERVO_DRIVES>> servoRx{};
    int rxBack = back_index(servoRx);
    servoRx.buffer[rxBack][0].motion.targetPosition = 123;
    publish(servoRx, rxBack);

    std::array<ServoRxPdo, MAX_SERVO_DRIVES> shadowRx{};
    read_snapshot(servoRx, shadowRx);
    if (shadowRx[0].motion.targetPosition != 123)
    {
        return 1;
    }

    DoubleBuffer<std::array<ServoTxPdo, MAX_SERVO_DRIVES>> servoTx{};
    int txBack = back_index(servoTx);
    servoTx.buffer[txBack][0].motion.positionActual = 77;
    publish(servoTx, txBack);

    std::array<ServoTxPdo, MAX_SERVO_DRIVES> txShadow{};
    read_snapshot(servoTx, txShadow);
    if (txShadow[0].motion.positionActual != 77)
    {
        return 1;
    }

    return 0;
}

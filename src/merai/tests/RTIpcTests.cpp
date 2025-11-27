#include <limits>
#include "merai/RTIpc.h"

int main()
{
    using namespace seven_axis_robot::merai;

    // DoubleBuffer publish/read path
    DoubleBuffer<int> buf{};
    int back = back_index(buf);
    buf.buffer[back] = 42;
    publish(buf, back);

    int value = 0;
    auto firstMeta = read_latest(buf, value, std::numeric_limits<uint64_t>::max());
    if (!firstMeta.fresh || firstMeta.seq != 1 || value != 42)
    {
        return 1;
    }

    auto secondMeta = read_latest(buf, value, firstMeta.seq);
    if (secondMeta.fresh)
    {
        return 1;
    }

    // Servo Rx/TX helpers and seq counters
    ServoBuffers servo{};
    int rxBack = servo_rx_back_index(servo);
    servo.buffer[rxBack].rx[0].motion.targetPosition = 123;
    publish_servo_rx(servo, rxBack);

    ServoSharedData shadow{};
    auto rxMeta = read_servo_rx(servo, shadow, 0);
    if (!rxMeta.fresh || rxMeta.seq != 1 || shadow.rx[0].motion.targetPosition != 123)
    {
        return 1;
    }

    auto rxMeta2 = read_servo_rx(servo, shadow, rxMeta.seq);
    if (rxMeta2.fresh)
    {
        return 1;
    }

    int txBack = servo_tx_back_index(servo);
    servo.buffer[txBack].tx[0].motion.positionActual = 77;
    publish_servo_tx(servo, txBack);

    ServoSharedData txShadow{};
    auto txMeta = read_servo_tx(servo, txShadow, 0);
    if (!txMeta.fresh || txMeta.seq != 1 || txShadow.tx[0].motion.positionActual != 77)
    {
        return 1;
    }

    return 0;
}

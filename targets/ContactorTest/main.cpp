#include "PVC/PVC.hpp"
#include <core/manager.hpp>
#include <PVC/dev/Contactor.hpp>

int main() {
    core::platform::init();

    PVC::Contactor cont(IO::getGPIO<PVC::PVC::CONT1_PIN>(),
                              IO::getGPIO<PVC::PVC::CONT2_PIN>());

    uint8_t i = 0;
    while (1) {
        cont.setOpen(i % 2);

        i++;
        core::time::wait(1000);
    }
}

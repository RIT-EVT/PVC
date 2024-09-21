#include "PVC/PVC.hpp"
#include <EVT/manager.hpp>
#include <PVC/dev/Contactor.hpp>

int main() {
    EVT::core::platform::init();

    PVC::Contactor cont(IO::getGPIO<PVC::PVC::CONT1_PIN>(),
                              IO::getGPIO<PVC::PVC::CONT2_PIN>());

    uint8_t i = 0;
    while (1) {
        cont.setOpen(i % 2);

        i++;
        EVT::core::time::wait(1000);
    }
}

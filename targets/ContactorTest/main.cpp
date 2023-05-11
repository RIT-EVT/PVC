#include <EVT/manager.hpp>
#include <PreCharge/PreCharge.hpp>
#include <PreCharge/dev/Contactor.hpp>

int main() {
    EVT::core::platform::init();

    PreCharge::Contactor cont(IO::getGPIO<PreCharge::PreCharge::CONT1_PIN>(),
                              IO::getGPIO<PreCharge::PreCharge::CONT2_PIN>());

    uint8_t i = 0;
    while (1) {
        cont.setOpen(i % 2);

        i++;
        EVT::core::time::wait(1000);
    }
}

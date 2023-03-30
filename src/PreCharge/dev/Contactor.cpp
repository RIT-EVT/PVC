#include <PreCharge/dev/Contactor.hpp>
#include <EVT/utils/time.hpp>

namespace time = EVT::core::time;

PreCharge::Contactor::Contactor(IO::GPIO& cont1, IO::GPIO& cont2) : cont1(cont1), cont2(cont2) {
    cont1.writePin(IO::GPIO::State::LOW);
    cont2.writePin(IO::GPIO::State::LOW);
}

void PreCharge::Contactor::setOpen(bool shouldOpen) {
    if (shouldOpen == isOpen) {
        return;
    }
    if (shouldOpen) {
        cont1.writePin(IO::GPIO::State::HIGH);
        time::wait(20);
        cont1.writePin(IO::GPIO::State::LOW);
    } else {
        cont2.writePin(IO::GPIO::State::HIGH);
        time::wait(20);
        cont2.writePin(IO::GPIO::State::LOW);
    }
    isOpen = shouldOpen;
}

bool PreCharge::Contactor::openState() {
    return isOpen;
}

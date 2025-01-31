#include <EVT/utils/time.hpp>
#include <PVC/dev/Contactor.hpp>

namespace time = EVT::core::time;

PVC::Contactor::Contactor(IO::GPIO& cont1, IO::GPIO& cont2) : cont1(cont1), cont2(cont2) {
    cont1.writePin(IO::GPIO::State::LOW);
    cont2.writePin(IO::GPIO::State::LOW);
}

void PVC::Contactor::setOpen(bool shouldOpen) {
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

bool PVC::Contactor::openState() {
    return isOpen;
}

#ifndef PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP
#define PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP

#include <EVT/io/GPIO.hpp>

namespace IO = EVT::core::IO;

namespace PreCharge {

class Contactor {
public:
    Contactor(IO::GPIO& cont1, IO::GPIO& cont2);

    void setOpen(bool shouldOpen);

    bool openState();

private:
    IO::GPIO& cont1;
    IO::GPIO& cont2;
    bool isOpen = true;
};

}// namespace PreCharge

#endif//PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP

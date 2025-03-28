#ifndef PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP
#define PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP

#include <core/io/GPIO.hpp>

namespace IO = core::io;

namespace PVC {

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

}// namespace PVC

#endif//PRE_CHARGE_INCLUDE_PRECHARGE_DEV_CONTACTOR_HPP

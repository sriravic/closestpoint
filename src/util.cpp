#include <util.h>

void
Timer::start()
{
    myStart = myclock.now();
}

void
Timer::stop()
{
    myEnd = myclock.now();
}

void
Timer::printElapsed(TimerUnits units) const
{
    auto printTime = [&](auto&& time, auto&& unitstr)
    {
        std::cout << "Timer : " << myName << " : " 
            << time << unitstr << std::endl;
    };

    switch (units)
    {
        case TimerUnits::TIMER_NS:
        {
            auto&& val = std::chrono::duration_cast<
                std::chrono::nanoseconds>(myEnd - myStart).count();
            printTime(val, "ns");
            break;
        }
        case TimerUnits::TIMER_MUS:
        {
            auto&& val = std::chrono::duration_cast<
                std::chrono::microseconds>(myEnd - myStart).count();
            printTime(val, "mus");
            break;
        }
        case TimerUnits::TIMER_MS:
        {
            auto&& val = std::chrono::duration_cast<
                std::chrono::milliseconds>(myEnd - myStart).count();
            printTime(val, "ms");
            break;
        }        
        case TimerUnits::TIMER_S:
        {
            auto&& val = std::chrono::duration_cast<
                std::chrono::seconds>(myEnd - myStart).count();
            printTime(val, "s");
            break;
        }
    }
}
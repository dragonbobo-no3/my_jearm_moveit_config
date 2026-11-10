#pragma once

namespace mk2_control
{
// The trajectory sample and its transmitted data time must describe the SAME
// instant. Do not fetch a fresh clock between update() and write(). Transport
// scheduling/watchdog checks remain independent of this sample timestamp.
template<class Manager, class Time, class Duration>
void run_cycle(Manager & manager, const Time & sample_time, const Duration & period)
{
  manager.read(sample_time, period);
  manager.update(sample_time, period);
  manager.write(sample_time, period);
}
}  // namespace mk2_control
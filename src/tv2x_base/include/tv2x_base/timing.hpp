#include <chrono>

namespace timing {
    using namespace std::chrono;

    enum TIME_MODULE {
        MILLISECONDS_MOD = 1000,
        NANOSECONDS_MOD = 1000000000,
    };

    struct Clock {
        template <class duration>
        static inline duration get_time() {
            return duration_cast<duration>(steady_clock::now().time_since_epoch());
        }
    };
};
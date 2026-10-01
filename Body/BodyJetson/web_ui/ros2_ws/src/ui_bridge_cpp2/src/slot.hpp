#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

namespace ui_bridge {

template <class T> class Slot {
public:
    using Clock = std::chrono::system_clock;

    struct Snapshot {
        std::shared_ptr<const T> msg;
        Clock::time_point recv_time;
    };

    void store(std::shared_ptr<const T> v) noexcept {
        std::lock_guard lk(m_);
        slot_ = std::move(v);
        recv_time_ = Clock::now();
        ++generation_;
    }

    std::shared_ptr<const T> load_if_newer(uint64_t &seen) const noexcept {
        std::lock_guard lk(m_);
        if (generation_ == seen)
            return nullptr;
        seen = generation_;
        return slot_;
    }

    Snapshot load() const noexcept {
        std::lock_guard lk(m_);
        return {slot_, recv_time_};
    }

private:
    mutable std::mutex m_;
    std::shared_ptr<const T> slot_;
    Clock::time_point recv_time_{};
    uint64_t generation_ = 0;
};

} // namespace ui_bridge
